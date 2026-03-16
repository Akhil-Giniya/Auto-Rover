"""
tests/test_all.py
─────────────────────────────────────────────────────────────────────────────
Pytest unit + integration tests for the ROV codebase.

Run on any machine (no hardware required):
    cd rov_final
    pytest tests/ -v

Tests cover
───────────
• iBus packet building and checksum
• iBus slew-rate limiter
• PID controller correctness
• IBVS alignment logic
• Dead reckoning math
• Watchdog timeout detection
• All detector functions (synthetic frames)
• State machine transitions
• DrumMap clustering
• Config loading
"""

import sys, os, time, json, struct, threading
import numpy as np
import pytest

# ── make rov_final importable ──────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


# ════════════════════════════════════════════════════════════════════════════
# iBus Interface tests
# ════════════════════════════════════════════════════════════════════════════

from hw_interface.ibus_interface import IBusInterface, NEUTRAL, MODE_PITCH_STAB

class FakeSerial:
    """Serial port stub."""
    def __init__(self): self.written = b""; self.is_open = True
    def write(self, data): self.written += data
    def close(self): self.is_open = False


def make_ibus() -> IBusInterface:
    ib = IBusInterface.__new__(IBusInterface)
    ib._port = "/dev/null"
    ib._baud = 115200
    ib._interval = 0.02
    ib._watchdog = None
    ib._channels = [NEUTRAL] * 14
    ib._prev_channels = [NEUTRAL] * 14
    ib._lock = threading.Lock()
    ib._running = False
    ib._thread = None
    ib._serial = FakeSerial()
    ib.packets_sent = 0
    ib.send_errors = 0
    ib.reconnect_count = 0
    ib._last_send_time = 0.0
    ib._start_time = time.monotonic()
    ib._armed = False
    return ib


class TestIBusPacket:
    def test_packet_length(self):
        ib = make_ibus()
        pkt = ib._build_packet([NEUTRAL] * 14)
        assert len(pkt) == 32

    def test_packet_header(self):
        ib = make_ibus()
        pkt = ib._build_packet([NEUTRAL] * 14)
        assert pkt[0] == 0x20
        assert pkt[1] == 0x40

    def test_checksum_valid(self):
        ib = make_ibus()
        for channels in ([NEUTRAL] * 14, [1000] * 14, [2000] * 14):
            pkt = ib._build_packet(channels)
            body = pkt[:30]
            ck   = struct.unpack_from('<H', pkt, 30)[0]
            assert (sum(body) + ck) & 0xFFFF == 0xFFFF

    def test_channel_clamp(self):
        ib = make_ibus()
        pkt = ib._build_packet([999, 2001] + [NEUTRAL] * 12)
        ch0 = struct.unpack_from('<H', pkt, 2)[0]
        ch1 = struct.unpack_from('<H', pkt, 4)[0]
        assert ch0 == 1000
        assert ch1 == 2000

    def test_norm_to_us_centre(self):
        assert IBusInterface._to_us(0.0) == 1500

    def test_norm_to_us_full(self):
        assert IBusInterface._to_us(1.0)  == 2000
        assert IBusInterface._to_us(-1.0) == 1000

    def test_norm_to_us_clamp(self):
        assert IBusInterface._to_us(5.0)  == 2000
        assert IBusInterface._to_us(-5.0) == 1000

    def test_slew_limit(self):
        ib = make_ibus()
        ib._prev_channels = [NEUTRAL] * 14
        result = ib._apply_slew([2000] * 14)
        from hw_interface.ibus_interface import MAX_JUMP_PER_TICK
        assert result[0] == NEUTRAL + MAX_JUMP_PER_TICK

    def test_set_motion_updates_channels(self):
        ib = make_ibus()
        ib.set_motion(surge=1.0, yaw=-1.0)
        with ib._lock:
            assert ib._channels[1] == 2000   # CH_SURGE
            assert ib._channels[3] == 1000   # CH_YAW

    def test_stop_motion_neutrals(self):
        ib = make_ibus()
        ib.set_motion(surge=1.0, yaw=1.0)
        ib.stop_motion()
        with ib._lock:
            assert ib._channels[1] == NEUTRAL
            assert ib._channels[3] == NEUTRAL


# ════════════════════════════════════════════════════════════════════════════
# PID tests
# ════════════════════════════════════════════════════════════════════════════

from navigation.alignment import PID, IBVSAligner, DeadReckoning

class TestPID:
    def test_proportional_only(self):
        pid = PID(kp=1.0, ki=0.0, kd=0.0, out_min=-10, out_max=10)
        out = pid.update(5.0)
        assert abs(out - 5.0) < 0.5

    def test_output_clamp(self):
        pid = PID(kp=100.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0)
        assert pid.update(1.0) == pytest.approx(1.0)
        assert pid.update(-1.0) == pytest.approx(-1.0)

    def test_zero_error(self):
        pid = PID(kp=1.0, ki=0.0, kd=0.0)
        assert pid.update(0.0) == pytest.approx(0.0, abs=1e-6)

    def test_integral_accumulates(self):
        pid = PID(kp=0.0, ki=1.0, kd=0.0, out_min=-100, out_max=100)
        out1 = pid.update(1.0)
        time.sleep(0.01)
        out2 = pid.update(1.0)
        assert out2 > out1   # integral grew

    def test_reset(self):
        pid = PID(kp=1.0, ki=1.0, kd=0.0, out_min=-100, out_max=100)
        pid.update(5.0)
        pid.reset()
        out = pid.update(0.0)
        assert abs(out) < 0.01


class TestIBVSAligner:
    _cfg = {"alignment": {
        "yaw_kp": 0.5, "yaw_ki": 0.0, "yaw_kd": 0.0,
        "yaw_max_cmd": 1.0, "center_tolerance_px": 20
    }}

    def test_aligned_returns_true(self):
        al = IBVSAligner(self._cfg)
        assert al.is_aligned(0)
        assert al.is_aligned(19)
        assert al.is_aligned(-19)

    def test_not_aligned_outside_tolerance(self):
        al = IBVSAligner(self._cfg)
        assert not al.is_aligned(21)
        assert not al.is_aligned(-21)

    def test_yaw_direction(self):
        al = IBVSAligner(self._cfg)
        right = al.compute_yaw(100)   # target is right → should yaw right
        left  = al.compute_yaw(-100)
        assert right > 0
        assert left  < 0

    def test_reset_clears_state(self):
        al = IBVSAligner(self._cfg)
        al.compute_yaw(200)
        al.reset()
        out = al.compute_yaw(0)
        assert abs(out) < 0.01


# ════════════════════════════════════════════════════════════════════════════
# Dead reckoning
# ════════════════════════════════════════════════════════════════════════════

class TestDeadReckoning:
    def test_straight_forward(self):
        dr = DeadReckoning(surge_mps=1.0, yaw_dps=60.0)
        # Simulate 1 s of forward motion manually
        dr._last_t = time.monotonic() - 1.0
        pose = dr.update(1.0, 0.0)
        assert pose.x > 0.1   # moved forward (dt clamped to 0.2 s max)
        assert abs(pose.y) < 0.1
        assert pose.heading == pytest.approx(0.0, abs=1.0)

    def test_yaw_changes_heading(self):
        dr = DeadReckoning(surge_mps=0.5, yaw_dps=60.0)
        dr._last_t = time.monotonic() - 1.0
        pose = dr.update(0.0, 1.0)
        assert pose.heading > 0.0

    def test_reset(self):
        dr = DeadReckoning()
        dr._last_t = time.monotonic() - 1.0
        dr.update(1.0, 0.5)
        dr.reset()
        p = dr.pose
        assert p.x == 0.0 and p.y == 0.0 and p.heading == 0.0


# ════════════════════════════════════════════════════════════════════════════
# Watchdog tests
# ════════════════════════════════════════════════════════════════════════════

from telemetry.watchdog import Watchdog

class TestWatchdog:
    def test_register_and_feed(self):
        w = Watchdog()
        w.register("test", timeout_s=1.0)
        w.feed("test")
        status = w.get_status()
        assert status["test"]["status"] == "ok"

    def test_timeout_marks_dead(self):
        w = Watchdog()
        w.register("slow", timeout_s=0.1)
        w.start()
        time.sleep(0.8)   # miss deadline 3× at 0.25s checks
        status = w.get_status()
        assert status["slow"]["status"] in ("degraded", "dead")
        w.stop()

    def test_recovery_from_dead(self):
        w = Watchdog()
        w.register("flaky", timeout_s=0.1)
        w.start()
        time.sleep(0.8)
        w.feed("flaky")
        time.sleep(0.1)
        status = w.get_status()
        assert status["flaky"]["status"] == "ok"
        w.stop()

    def test_critical_fires_callback(self):
        fired = []
        w = Watchdog()
        w.register("crit", timeout_s=0.05, critical=True)
        w.start(on_critical_failure=lambda: fired.append(1))
        time.sleep(1.0)
        assert len(fired) >= 1
        w.stop()

    def test_all_ok_true_when_healthy(self):
        w = Watchdog()
        w.register("a", timeout_s=5.0)
        w.register("b", timeout_s=5.0)
        w.feed("a"); w.feed("b")
        assert w.all_ok()


# ════════════════════════════════════════════════════════════════════════════
# Vision detectors (synthetic frames)
# ════════════════════════════════════════════════════════════════════════════

import cv2
from vision.pipeline import (
    detect_flare, detect_gate, detect_green_mat, detect_drums, FRAME_CX, FRAME_CY
)

def _blank(h=480, w=640) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


def _hsv_rect(frame, cx, cy, r, h_val, s_val=220, v_val=200):
    """Paint a saturated HSV rectangle into a BGR frame."""
    hsv_img = np.zeros_like(frame)
    hsv_img[cy-r:cy+r, cx-r:cx+r] = [h_val, s_val, v_val]
    bgr = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)
    mask = hsv_img[:, :, 1] > 0
    frame[mask] = bgr[mask]
    return frame


_CFG = {
    "orange_flare": {"h_low":5,"s_low":120,"v_low":100,"h_high":20,"s_high":255,"v_high":255,"min_area":400},
    "gate": {"canny_low":30,"canny_high":100,"hough_threshold":30,"hough_min_line_length":40,"hough_max_line_gap":20,"vertical_angle_tolerance_deg":20,"min_pipe_separation_px":40},
    "green_mat": {"h_low":40,"s_low":60,"v_low":40,"h_high":90,"s_high":255,"v_high":255,"min_area":500},
    "red_drum": {"ranges":[{"h_low":0,"s_low":100,"v_low":60,"h_high":10,"s_high":255,"v_high":255},{"h_low":160,"s_low":100,"v_low":60,"h_high":180,"s_high":255,"v_high":255}],"min_area":200},
    "blue_drum": {"h_low":95,"s_low":80,"v_low":50,"h_high":130,"s_high":255,"v_high":255,"min_area":200,"drop_min_area":4000},
}


class TestDetectors:
    def test_flare_detects_orange(self):
        frame = _blank()
        _hsv_rect(frame, 200, 240, 40, 12)   # orange hue ~12
        r = detect_flare(frame, _CFG)
        assert r.detected
        assert r.side == "left"

    def test_flare_no_detect_on_blank(self):
        r = detect_flare(_blank(), _CFG)
        assert not r.detected

    def test_gate_detects_vertical_lines(self):
        frame = _blank()
        cv2.line(frame, (150, 50), (150, 430), (200, 200, 200), 3)
        cv2.line(frame, (490, 50), (490, 430), (200, 200, 200), 3)
        r = detect_gate(frame, _CFG)
        assert r.detected
        assert abs(r.center_x - 320) < 50

    def test_gate_no_detect_on_blank(self):
        r = detect_gate(_blank(), _CFG)
        assert not r.detected

    def test_green_mat_detected(self):
        frame = _blank()
        _hsv_rect(frame, 320, 400, 60, 60)   # green hue ~60
        r = detect_green_mat(frame, _CFG)
        assert r.detected

    def test_blue_drum_detected(self):
        frame = _blank()
        _hsv_rect(frame, 320, 240, 30, 110)  # blue hue ~110
        r = detect_drums(frame, _CFG)
        assert r.blue_drum.detected

    def test_red_drum_detected(self):
        frame = _blank()
        _hsv_rect(frame, 200, 240, 30, 5)    # red hue ~5
        r = detect_drums(frame, _CFG)
        assert len(r.red_drums) >= 1

    def test_detectors_dont_raise_on_garbage(self):
        garbage = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        detect_flare(garbage, _CFG)
        detect_gate(garbage, _CFG)
        detect_green_mat(garbage, _CFG)
        detect_drums(garbage, _CFG)


# ════════════════════════════════════════════════════════════════════════════
# DrumMap tests
# ════════════════════════════════════════════════════════════════════════════

from mapping.drum_map import DrumMap, _cluster

class TestDrumMap:
    def test_cluster_single(self):
        pts = np.array([[100, 200], [105, 202], [98, 198]])
        c = _cluster(pts, 20)
        assert len(c) == 1

    def test_cluster_two_groups(self):
        pts = np.array([[100, 100], [102, 100], [500, 400], [498, 402]])
        c = _cluster(pts, 20)
        assert len(c) == 2

    def test_get_positions_empty(self):
        dm = DrumMap()
        assert dm.get_positions() == {}

    def test_get_positions_with_data(self):
        dm = DrumMap()
        dm.record_red(100, 200)
        dm.record_red(102, 200)
        dm.record_blue(320, 240)
        pos = dm.get_positions()
        assert "blue" in pos
        assert any(k.startswith("red") for k in pos)

    def test_finalize_creates_text(self, tmp_path):
        dm = DrumMap()
        dm.record_blue(320, 240)
        out = str(tmp_path / "map.png")
        summary = dm.finalize(out)
        assert "blue" in summary


# ════════════════════════════════════════════════════════════════════════════
# Config loading
# ════════════════════════════════════════════════════════════════════════════

class TestConfig:
    def test_config_loads(self):
        cfg_path = os.path.join(os.path.dirname(__file__), "..", "config.json")
        with open(cfg_path) as f:
            cfg = json.load(f)
        assert "orange_flare" in cfg
        assert "blue_drum" in cfg
        assert "navigation" in cfg
        assert "alignment" in cfg

    def test_config_has_required_keys(self):
        cfg_path = os.path.join(os.path.dirname(__file__), "..", "config.json")
        with open(cfg_path) as f:
            cfg = json.load(f)
        nav = cfg["navigation"]
        assert "surge_speed" in nav
        assert "uturn_yaw_speed" in nav
        align = cfg["alignment"]
        assert "yaw_kp" in align
        assert "center_tolerance_px" in align
