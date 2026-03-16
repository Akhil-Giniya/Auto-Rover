"""
hw_interface/ibus_interface.py
─────────────────────────────────────────────────────────────────────────────
Production iBus interface to the ESP32.

Robustness features
───────────────────
• Auto-reconnect on serial port failure (up to MAX_RECONNECT attempts)
• Watchdog feed on every successful packet — watchdog fires failsafe if link dies
• Health metrics: packets_sent, errors, last_send_age_ms
• Slew-rate guard on RPi side (don't jump channels > MAX_JUMP_PER_TICK)
• Emergency-stop: zeros all motion channels, sends 10 disarm packets
• Thread-safe channel access

iBus packet format (32 bytes):
  [0]    0x20  (length)
  [1]    0x40  (command)
  [2-29] 14 × uint16_le channels (1000–2000)
  [30-31] uint16_le checksum = 0xFFFF − sum(bytes 0..29)
"""

import struct
import threading
import time
import serial
import serial.tools.list_ports
from typing import Optional

from rov_logger import get_logger

log = get_logger("hw_interface")

# ── Channel index constants (from ESP32 source, 0-based) ──────────────────
CH_ROLL_SWAY   = 0   # RC_CH_ROLL_SWAY   — keep 1500 (neutral) in auto
CH_SURGE       = 1   # RC_CH_PITCH_SURGE — forward/reverse
CH_HEAVE       = 2   # RC_CH_HEAVE       — up/down
CH_YAW         = 3   # RC_CH_YAW         — yaw rate
CH_SERVO_FRONT = 4   # RC_CH_SERVO_FRONT — ball drop servo
CH_MODE        = 5   # RC_CH_MODE        — 2000=PITCH_STAB ← must always be set
CH_SERVO_REAR  = 6   # RC_CH_SERVO_REAR  — keep 1500
CH_SWAY_SEL    = 7   # RC_CH_SWAY_SEL    — 2000=CH1 becomes sway axis
CH_PITCH_SEL   = 8   # RC_CH_PITCH_SEL   — 1000=CH2 becomes surge axis
CH_ARM         = 9   # RC_CH_ARM         — 2000=armed

NEUTRAL          = 1500
MODE_PITCH_STAB  = 2000
ARMED            = 2000
DISARMED         = 1000
BALL_OPEN        = 2000
BALL_CLOSED      = 1000

MAX_RECONNECT    = 10
RECONNECT_DELAY  = 2.0    # seconds between retries
MAX_JUMP_PER_TICK = 30    # µs slew limit per send tick (safety)
SEND_RATE_HZ     = 50
ESP32_FAILSAFE_MS = 500   # ESP32 fires failsafe if no packet for this long


class IBusError(Exception):
    """Raised for unrecoverable iBus failures."""


class IBusInterface:

    IBUS_LENGTH  = 0x20
    IBUS_COMMAND = 0x40

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 rate_hz: float = SEND_RATE_HZ,
                 watchdog=None):
        self._port      = port
        self._baud      = baud
        self._interval  = 1.0 / rate_hz
        self._watchdog  = watchdog

        self._channels       = [NEUTRAL] * 14
        self._prev_channels  = [NEUTRAL] * 14
        self._lock           = threading.Lock()
        self._running        = False
        self._thread: Optional[threading.Thread] = None
        self._serial: Optional[serial.Serial] = None

        # Metrics
        self.packets_sent    = 0
        self.send_errors     = 0
        self.reconnect_count = 0
        self._last_send_time = 0.0
        self._start_time     = 0.0

        # Arm/disarm state
        self._armed          = False

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self, armed: bool = False) -> None:
        self._armed = armed
        self._start_time = time.monotonic()
        self._setup_startup_channels(armed)
        self._open_serial()
        self._running = True
        self._thread = threading.Thread(
            target=self._send_loop, daemon=True, name="ibus-tx"
        )
        self._thread.start()
        log.info(f"iBus started: port={self._port} baud={self._baud} armed={armed}")

    def stop(self) -> None:
        """Graceful stop: zero motion, disarm, send 10 packets, close."""
        log.info("iBus stopping — disarming")
        self._running = False

        with self._lock:
            for ch in (CH_SURGE, CH_HEAVE, CH_YAW, CH_ROLL_SWAY):
                self._channels[ch] = NEUTRAL
            self._channels[CH_ARM] = DISARMED

        # Flush final disarm packets
        for _ in range(10):
            self._send_once()
            time.sleep(0.02)

        if self._thread:
            self._thread.join(timeout=3.0)
        self._close_serial()
        log.info(f"iBus stopped — packets_sent={self.packets_sent} errors={self.send_errors}")

    def emergency_stop(self) -> None:
        """Immediate stop — called by watchdog or exception handlers."""
        log.critical("iBus EMERGENCY STOP")
        with self._lock:
            self._channels = [NEUTRAL] * 14
            self._channels[CH_MODE] = MODE_PITCH_STAB
            self._channels[CH_ARM]  = DISARMED
        for _ in range(10):
            try:
                self._send_once()
            except Exception:
                pass
            time.sleep(0.02)

    # ── Motion API ────────────────────────────────────────────────────────

    def set_motion(self, surge: float = 0.0, heave: float = 0.0,
                   yaw: float = 0.0, sway: float = 0.0) -> None:
        """
        All values normalised −1.0 … +1.0.
        Clamped and slew-limited before writing to channels.
        """
        with self._lock:
            self._channels[CH_SURGE]     = self._to_us(surge)
            self._channels[CH_HEAVE]     = self._to_us(heave)
            self._channels[CH_YAW]       = self._to_us(yaw)
            self._channels[CH_ROLL_SWAY] = self._to_us(sway)

    def stop_motion(self) -> None:
        with self._lock:
            for ch in (CH_SURGE, CH_HEAVE, CH_YAW, CH_ROLL_SWAY):
                self._channels[ch] = NEUTRAL

    def arm(self) -> None:
        with self._lock:
            self._channels[CH_ARM] = ARMED
            self._armed = True
        log.info("iBus: ARMED")

    def disarm(self) -> None:
        with self._lock:
            self._channels[CH_ARM] = DISARMED
            self._armed = False
        log.info("iBus: DISARMED")

    def set_ball_drop(self, open_: bool) -> None:
        val = BALL_OPEN if open_ else BALL_CLOSED
        with self._lock:
            self._channels[CH_SERVO_FRONT] = val
        log.info(f"Ball drop servo: {'OPEN' if open_ else 'CLOSED'}")

    def set_channel_raw(self, idx: int, us: int) -> None:
        us = max(1000, min(2000, us))
        with self._lock:
            self._channels[idx] = us

    # ── Properties ────────────────────────────────────────────────────────

    @property
    def last_send_age_ms(self) -> float:
        return (time.monotonic() - self._last_send_time) * 1000.0

    @property
    def is_healthy(self) -> bool:
        return self.last_send_age_ms < ESP32_FAILSAFE_MS

    @property
    def uptime_s(self) -> float:
        return time.monotonic() - self._start_time

    def get_channels_snapshot(self) -> list[int]:
        with self._lock:
            return list(self._channels)

    # ── Internal ──────────────────────────────────────────────────────────

    def _setup_startup_channels(self, armed: bool) -> None:
        with self._lock:
            self._channels = [NEUTRAL] * 14
            self._channels[CH_MODE]     = MODE_PITCH_STAB
            self._channels[CH_SWAY_SEL] = 2000   # CH1 = sway axis
            self._channels[CH_PITCH_SEL]= 1000   # CH2 = surge axis
            self._channels[CH_ARM]      = ARMED if armed else DISARMED
            self._channels[CH_SERVO_FRONT] = BALL_CLOSED

    def _open_serial(self) -> None:
        for attempt in range(1, MAX_RECONNECT + 1):
            try:
                if self._serial and self._serial.is_open:
                    self._serial.close()
                self._serial = serial.Serial(
                    self._port, self._baud,
                    timeout=0.1, write_timeout=0.2
                )
                log.info(f"Serial opened: {self._port} (attempt {attempt})")
                return
            except serial.SerialException as e:
                log.error(f"Serial open failed (attempt {attempt}/{MAX_RECONNECT}): {e}")
                if attempt < MAX_RECONNECT:
                    time.sleep(RECONNECT_DELAY)
                else:
                    raise IBusError(f"Cannot open {self._port} after {MAX_RECONNECT} attempts") from e

    def _close_serial(self) -> None:
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
                log.info("Serial port closed")
        except Exception as e:
            log.warning(f"Serial close error: {e}")

    def _build_packet(self, channels: list[int]) -> bytes:
        buf = bytearray([self.IBUS_LENGTH, self.IBUS_COMMAND])
        for ch in channels:
            buf += struct.pack('<H', max(1000, min(2000, ch)))
        checksum = 0xFFFF - (sum(buf) & 0xFFFF)
        buf += struct.pack('<H', checksum & 0xFFFF)
        return bytes(buf)

    def _apply_slew(self, channels: list[int]) -> list[int]:
        """Limit per-tick channel jumps to MAX_JUMP_PER_TICK."""
        result = list(channels)
        for i in range(len(channels)):
            delta = channels[i] - self._prev_channels[i]
            if abs(delta) > MAX_JUMP_PER_TICK:
                result[i] = self._prev_channels[i] + MAX_JUMP_PER_TICK * (1 if delta > 0 else -1)
        self._prev_channels = list(result)
        return result

    def _send_once(self) -> bool:
        try:
            with self._lock:
                raw = list(self._channels)
            smoothed = self._apply_slew(raw)
            packet = self._build_packet(smoothed)
            self._serial.write(packet)
            self._last_send_time = time.monotonic()
            self.packets_sent += 1
            if self._watchdog:
                self._watchdog.feed("ibus")
            return True
        except serial.SerialTimeoutException as e:
            self.send_errors += 1
            log.warning(f"iBus write timeout: {e}")
            if self._watchdog:
                self._watchdog.increment_error("ibus")
            return False
        except serial.SerialException as e:
            self.send_errors += 1
            log.error(f"iBus serial error: {e} — attempting reconnect")
            self._attempt_reconnect()
            return False
        except Exception as e:
            self.send_errors += 1
            log.error(f"iBus unexpected error: {e}", exc_info=True)
            return False

    def _attempt_reconnect(self) -> None:
        self.reconnect_count += 1
        log.warning(f"iBus reconnect attempt #{self.reconnect_count}")
        try:
            self._open_serial()
            log.info("iBus reconnected successfully")
        except IBusError:
            log.critical("iBus reconnect failed — triggering emergency stop")
            self.emergency_stop()

    def _send_loop(self) -> None:
        log.debug("iBus send loop started")
        while self._running:
            t0 = time.monotonic()
            self._send_once()
            elapsed = time.monotonic() - t0
            remaining = self._interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    @staticmethod
    def _to_us(v: float) -> int:
        return int(NEUTRAL + max(-1.0, min(1.0, v)) * 500)
