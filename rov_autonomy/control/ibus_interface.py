"""
control/ibus_interface.py
Sends iBus packets to the ESP32 over UART.

iBus packet = 0x20 0x40 + 14 channels × 2 bytes (little-endian) + 2-byte checksum
Each channel: 1000–2000 µs  (1500 = neutral)

ESP32 index mapping (from Esp32code_.txt):
  idx 0  CH1  RC_CH_ROLL_SWAY      → keep 1500 (neutral)
  idx 1  CH2  RC_CH_PITCH_SURGE    → forward/reverse  (RPi controls)
  idx 2  CH3  RC_CH_HEAVE          → up/down          (RPi controls)
  idx 3  CH4  RC_CH_YAW            → yaw rate         (RPi controls)
  idx 4  CH5  RC_CH_SERVO_FRONT    → keep 1500 (ESP32 auto-angles in PITCH_STAB)
  idx 5  CH6  RC_CH_MODE           → 2000 = PITCH_STAB  ← CRITICAL
  idx 6  CH7  RC_CH_SERVO_REAR     → keep 1500
  idx 7  CH8  RC_CH_SWAY_SEL       → 2000 = sway mode on CH1 (set once)
  idx 8  CH9  RC_CH_PITCH_SEL      → 1000 = surge mode on CH2 (set once)
  idx 9  CH10 RC_CH_ARM            → 2000 = armed        ← CRITICAL
  idx 10–13  unused, keep 1500

IMPORTANT: If the ESP32 stops receiving valid iBus frames for >500 ms it
enters failsafe (DEPTH_STAB, all commands zeroed, thrusters to neutral).
The caller must call send() at ≥ 20 Hz continuously.
"""

import struct
import threading
import time
import serial


# ──────────────────────────────────────────────────────────────────────────────
# Channel indices (0-based, matching ESP32 source)
# ──────────────────────────────────────────────────────────────────────────────
CH_ROLL_SWAY   = 0
CH_SURGE       = 1   # RC_CH_PITCH_SURGE → forward/reverse
CH_HEAVE       = 2   # up/down
CH_YAW         = 3   # yaw rate setpoint
CH_SERVO_FRONT = 4   # keep 1500 in auto modes
CH_MODE        = 5   # 1000=MANUAL 1200-1699=DEPTH_STAB ≥1700=PITCH_STAB
CH_SERVO_REAR  = 6   # keep 1500 in auto modes
CH_SWAY_SEL    = 7   # 2000 → CH1 becomes sway
CH_PITCH_SEL   = 8   # 1000 → CH2 becomes surge  (we want surge)
CH_ARM         = 9   # 2000 = armed

NEUTRAL = 1500
MODE_PITCH_STAB = 2000   # ≥1700 → PITCH_STAB in ESP32
MODE_DEPTH_STAB = 1500   # 1300–1699 → DEPTH_STAB
MODE_MANUAL     = 1000   # <1300 → MANUAL
ARMED           = 2000
DISARMED        = 1000


class IBusInterface:
    """
    Thread-safe iBus sender.
    Background thread sends the current channel values at `rate_hz`.
    RPi autonomy code updates channels via set_channel() or set_motion().
    """

    IBUS_LENGTH  = 0x20   # 32 bytes total
    IBUS_COMMAND = 0x40

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 rate_hz: float = 50.0):
        self._port = port
        self._baud = baud
        self._rate_hz = rate_hz
        self._interval = 1.0 / rate_hz

        # 14 channels, all neutral to start
        self._channels = [NEUTRAL] * 14
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._serial = None
        self._last_send_time = 0.0

    # ──────────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────────
    def start(self, armed: bool = False) -> None:
        """Open serial port and start background send loop."""
        self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
        time.sleep(0.1)

        # Set safe startup values
        self._channels = [NEUTRAL] * 14
        self._channels[CH_MODE]      = MODE_PITCH_STAB  # stabilised
        self._channels[CH_SWAY_SEL]  = 2000             # CH1 = sway axis
        self._channels[CH_PITCH_SEL] = 1000             # CH2 = surge axis
        self._channels[CH_ARM]       = ARMED if armed else DISARMED

        self._running = True
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()
        print(f"[iBus] started on {self._port} @ {self._baud} baud, "
              f"{self._rate_hz} Hz, armed={armed}")

    def stop(self) -> None:
        """Stop send loop, zero all motion, then close port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        # Send a few disarmed neutral packets before closing
        with self._lock:
            self._channels = [NEUTRAL] * 14
            self._channels[CH_MODE] = MODE_PITCH_STAB
            self._channels[CH_ARM]  = DISARMED
        for _ in range(5):
            self._send_once()
            time.sleep(0.02)
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("[iBus] stopped")

    # ──────────────────────────────────────────────────────────────────
    # Motion commands  (values −1.0 … +1.0)
    # ──────────────────────────────────────────────────────────────────
    def set_motion(self, surge: float = 0.0, heave: float = 0.0,
                   yaw: float = 0.0, sway: float = 0.0) -> None:
        """
        surge: +1 = full forward,  −1 = full reverse
        heave: +1 = full up,       −1 = full down
        yaw:   +1 = turn right,    −1 = turn left
        sway:  +1 = slide right,   −1 = slide left
              (only works when CH8 sway mode = 2000)
        """
        with self._lock:
            self._channels[CH_SURGE] = self._norm_to_us(surge)
            self._channels[CH_HEAVE] = self._norm_to_us(heave)
            self._channels[CH_YAW]   = self._norm_to_us(yaw)
            # For sway: CH8 is already set to 2000, drive CH1
            self._channels[CH_ROLL_SWAY] = self._norm_to_us(sway)

    def stop_motion(self) -> None:
        """Zero all motion channels. ESP32 slew-rates to stop."""
        self.set_motion(0.0, 0.0, 0.0, 0.0)

    def arm(self) -> None:
        with self._lock:
            self._channels[CH_ARM] = ARMED
        print("[iBus] ARMED")

    def disarm(self) -> None:
        with self._lock:
            self._channels[CH_ARM] = DISARMED
        print("[iBus] DISARMED")

    def set_channel(self, idx: int, value: int) -> None:
        """Raw channel write. value must be 1000–2000."""
        value = max(1000, min(2000, value))
        with self._lock:
            self._channels[idx] = value

    def get_channel(self, idx: int) -> int:
        with self._lock:
            return self._channels[idx]

    # ──────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _norm_to_us(v: float) -> int:
        """Convert normalised −1…+1 to 1000–2000 µs."""
        v = max(-1.0, min(1.0, v))
        return int(NEUTRAL + v * 500)

    def _build_packet(self) -> bytes:
        """Build 32-byte iBus packet from current channel array."""
        # Header: length byte + command byte
        buf = bytearray([self.IBUS_LENGTH, self.IBUS_COMMAND])
        with self._lock:
            channels = list(self._channels)
        for ch in channels:
            buf += struct.pack('<H', max(1000, min(2000, ch)))
        # Checksum = 0xFFFF − sum of all bytes
        checksum = 0xFFFF - sum(buf)
        buf += struct.pack('<H', checksum & 0xFFFF)
        return bytes(buf)

    def _send_once(self) -> None:
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(self._build_packet())
                self._last_send_time = time.monotonic()
        except serial.SerialException as e:
            print(f"[iBus] serial error: {e}")

    def _send_loop(self) -> None:
        while self._running:
            start = time.monotonic()
            self._send_once()
            elapsed = time.monotonic() - start
            sleep_time = self._interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    @property
    def last_send_age_ms(self) -> float:
        """Milliseconds since last successful packet. >500 ms → ESP32 failsafe."""
        return (time.monotonic() - self._last_send_time) * 1000.0
