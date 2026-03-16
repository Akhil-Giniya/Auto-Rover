"""
autonomy_core/state_machine.py
─────────────────────────────────────────────────────────────────────────────
Autonomous mission state machine — production hardened.

Robustness features
───────────────────
• Every state has a hard timeout → graceful skip-forward (never hangs)
• Exception in any state tick → logged, motion stopped, state aborted
• Watchdog feed on every tick
• All state transitions logged with timestamps
• Emergency-stop path: any CRITICAL event → SAFE_STOP state
• Telemetry published on every state change and every tick
• State history recorded for post-run analysis
"""

import time
import threading
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional, Callable

from rov_logger import get_logger
from telemetry.bus import TelemetryPublisher, StateMsg, MotionMsg, ErrorMsg
from telemetry.watchdog import Watchdog
from hw_interface.ibus_interface import IBusInterface
from vision.pipeline import VisionPipeline, PipelineResult
from navigation.alignment import IBVSAligner, DeadReckoning
from mapping.drum_map import DrumMap

log = get_logger("autonomy_core")


# ── States ────────────────────────────────────────────────────────────────

class State(IntEnum):
    IDLE             = 0
    ENTER_ARENA      = 1
    SEARCH_GATE      = 2
    PASS_GATE_FWD    = 3
    U_TURN           = 4
    PASS_GATE_REV    = 5
    SEARCH_GREEN_MAT = 6
    DETECT_DRUMS     = 7
    ALIGN_BLUE_DRUM  = 8
    DROP_BALL        = 9
    GENERATE_MAP     = 10
    FIND_EXIT        = 11
    MISSION_COMPLETE = 12
    SAFE_STOP        = 99   # emergency state


# ── State configuration ───────────────────────────────────────────────────

@dataclass
class StateConfig:
    timeout_s:       float        # hard timeout → auto advance
    next_on_timeout: "State"      # where to go if timeout fires
    active_detectors: set          # which detectors to run
    description:     str = ""

STATE_CONFIGS: dict[State, StateConfig] = {
    State.ENTER_ARENA:      StateConfig(4.0,  State.SEARCH_GATE,      {"flare"},            "Drive through start gate"),
    State.SEARCH_GATE:      StateConfig(20.0, State.PASS_GATE_FWD,    {"flare", "gate"},    "Find and align to gate"),
    State.PASS_GATE_FWD:    StateConfig(4.0,  State.U_TURN,           {"flare"},            "Pass through gate forward"),
    State.U_TURN:           StateConfig(5.0,  State.PASS_GATE_REV,    set(),                "180° U-turn"),
    State.PASS_GATE_REV:    StateConfig(4.0,  State.SEARCH_GREEN_MAT, {"gate"},             "Pass through gate reverse"),
    State.SEARCH_GREEN_MAT: StateConfig(20.0, State.DETECT_DRUMS,     {"green_mat"},        "Find green mat"),
    State.DETECT_DRUMS:     StateConfig(20.0, State.ALIGN_BLUE_DRUM,  {"drums"},            "Detect and map drums"),
    State.ALIGN_BLUE_DRUM:  StateConfig(20.0, State.DROP_BALL,        {"drums"},            "Align to blue drum"),
    State.DROP_BALL:        StateConfig(3.0,  State.GENERATE_MAP,     set(),                "Drop ball into drum"),
    State.GENERATE_MAP:     StateConfig(3.0,  State.FIND_EXIT,        set(),                "Generate drum map"),
    State.FIND_EXIT:        StateConfig(8.0,  State.MISSION_COMPLETE, set(),                "Navigate to exit"),
    State.MISSION_COMPLETE: StateConfig(99.0, State.MISSION_COMPLETE, set(),                "Mission done"),
    State.SAFE_STOP:        StateConfig(99.0, State.SAFE_STOP,        set(),                "Emergency safe stop"),
    State.IDLE:             StateConfig(99.0, State.IDLE,             set(),                "Idle"),
}


# ── State history entry ───────────────────────────────────────────────────

@dataclass
class StateRecord:
    state:     State
    entered:   float
    exited:    float = 0.0
    reason:    str   = ""


# ── Main FSM ─────────────────────────────────────────────────────────────

class MissionStateMachine:

    def __init__(self,
                 ibus:     IBusInterface,
                 vision:   VisionPipeline,
                 cfg:      dict,
                 watchdog: Optional[Watchdog] = None,
                 publisher: Optional[TelemetryPublisher] = None,
                 debug_display: bool = False):
        self._ibus     = ibus
        self._vision   = vision
        self._cfg      = cfg
        self._wdog     = watchdog
        self._pub      = publisher
        self._debug    = debug_display

        self._aligner   = IBVSAligner(cfg)
        self._dead_reck = DeadReckoning(
            surge_mps=cfg["navigation"].get("surge_mps", 0.4),
            yaw_dps=cfg["navigation"].get("yaw_dps", 55.0)
        )
        self._drum_map  = DrumMap()

        self._state     = State.IDLE
        self._prev_state = State.IDLE
        self._state_start = time.monotonic()
        self._running   = False
        self._history:  list[StateRecord] = []
        self._tick_count = 0
        self._loop_hz    = 0.0
        self._hz_frames  = 0
        self._hz_t0      = time.monotonic()

        # Navigation values for telemetry
        self._last_surge = 0.0
        self._last_yaw   = 0.0
        self._last_heave = 0.0
        self._last_sway  = 0.0

    # ── Public API ────────────────────────────────────────────────────────

    def run(self) -> None:
        """Blocking mission loop."""
        self._running = True
        log.info("=" * 60)
        log.info("MISSION START")
        log.info("=" * 60)
        self._enter_state(State.ENTER_ARENA)

        while self._running:
            t0 = time.monotonic()
            try:
                self._tick()
            except Exception as e:
                log.critical(f"State machine tick exception in {self._state.name}: {e}",
                             exc_info=True)
                self._pub_error("TICK_EXCEPTION", str(e), "critical")
                self._safe_stop("Tick exception")

            # Feed watchdog
            if self._wdog:
                self._wdog.feed("autonomy")

            # Track loop Hz
            self._tick_count += 1
            self._hz_frames  += 1
            elapsed = time.monotonic() - self._hz_t0
            if elapsed >= 2.0:
                self._loop_hz   = self._hz_frames / elapsed
                self._hz_frames = 0
                self._hz_t0     = time.monotonic()

            # Maintain ~25 Hz
            dt = time.monotonic() - t0
            sleep = max(0.0, 0.04 - dt)
            if sleep > 0:
                time.sleep(sleep)

        self._shutdown()

    def emergency_stop(self) -> None:
        """Called by watchdog on critical failure."""
        self._safe_stop("Watchdog triggered emergency stop")

    def stop(self) -> None:
        self._running = False

    # ── Internal tick ─────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._state in (State.MISSION_COMPLETE, State.SAFE_STOP, State.IDLE):
            return

        result = self._vision.get_result()
        age = self._state_age()
        sc  = STATE_CONFIGS[self._state]

        # Hard timeout guard
        if age >= sc.timeout_s:
            log.warning(
                f"State {self._state.name} timed out after {age:.1f}s "
                f"(limit={sc.timeout_s}s) → {sc.next_on_timeout.name}"
            )
            self._pub_error("STATE_TIMEOUT", f"{self._state.name} @ {age:.1f}s", "warning")
            self._transition(sc.next_on_timeout, "timeout")
            return

        # Camera health check
        if not result.ok:
            log.error("Vision pipeline not ok — stopping motion")
            self._stop_motion()
            if self._wdog:
                self._wdog.set_degraded("vision", "pipeline failed")
            return

        # ── Flare avoidance (active in states 1–3) ──
        if self._state in (State.ENTER_ARENA, State.SEARCH_GATE, State.PASS_GATE_FWD):
            if result.flare.detected:
                self._avoid_flare(result.flare)
                return

        # ── State dispatch ──
        dispatch = {
            State.ENTER_ARENA:      self._s_enter_arena,
            State.SEARCH_GATE:      self._s_search_gate,
            State.PASS_GATE_FWD:    self._s_pass_gate_fwd,
            State.U_TURN:           self._s_u_turn,
            State.PASS_GATE_REV:    self._s_pass_gate_rev,
            State.SEARCH_GREEN_MAT: self._s_search_green_mat,
            State.DETECT_DRUMS:     self._s_detect_drums,
            State.ALIGN_BLUE_DRUM:  self._s_align_blue_drum,
            State.DROP_BALL:        self._s_drop_ball,
            State.GENERATE_MAP:     self._s_generate_map,
            State.FIND_EXIT:        self._s_find_exit,
        }
        fn = dispatch.get(self._state)
        if fn:
            fn(result, age)

    # ── State handlers ────────────────────────────────────────────────────

    def _s_enter_arena(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        self._set_motion(surge=n["surge_speed"])
        if age >= STATE_CONFIGS[State.ENTER_ARENA].timeout_s * 0.9:
            self._transition(State.SEARCH_GATE, "timer")

    def _s_search_gate(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        if r.gate.detected:
            yaw = self._aligner.compute_yaw(r.gate.error_x)
            self._set_motion(surge=n["approach_speed"], yaw=yaw)
            if self._aligner.is_aligned(r.gate.error_x):
                log.info(f"Gate aligned (error_x={r.gate.error_x})")
                self._transition(State.PASS_GATE_FWD, "gate aligned")
        else:
            self._set_motion(surge=0.12)   # slow search sweep

    def _s_pass_gate_fwd(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        self._set_motion(surge=n["surge_speed"])
        if age >= STATE_CONFIGS[State.PASS_GATE_FWD].timeout_s * 0.9:
            self._transition(State.U_TURN, "through gate")

    def _s_u_turn(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        self._set_motion(yaw=n["uturn_yaw_speed"])
        if age >= STATE_CONFIGS[State.U_TURN].timeout_s * 0.9:
            self._transition(State.PASS_GATE_REV, "u-turn done")

    def _s_pass_gate_rev(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        yaw = self._aligner.compute_yaw(r.gate.error_x) if r.gate.detected else 0.0
        self._set_motion(surge=n["surge_speed"], yaw=yaw)
        if age >= STATE_CONFIGS[State.PASS_GATE_REV].timeout_s * 0.9:
            self._transition(State.SEARCH_GREEN_MAT, "passed gate reverse")

    def _s_search_green_mat(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        if r.green_mat.detected:
            yaw = self._aligner.compute_yaw(r.green_mat.cx - 320)
            self._set_motion(surge=n["approach_speed"], yaw=yaw)
            if r.green_mat.area > self._cfg["green_mat"]["min_area"] * 4:
                self._transition(State.DETECT_DRUMS, "on green mat")
        else:
            self._set_motion(surge=0.12)

    def _s_detect_drums(self, r: PipelineResult, age: float):
        drums = r.drums
        for d in drums.red_drums:
            self._drum_map.record_red(d.cx, d.cy)
        if drums.blue_drum.detected:
            self._drum_map.record_blue(drums.blue_drum.cx, drums.blue_drum.cy)
            self._transition(State.ALIGN_BLUE_DRUM, "blue drum found")
            return
        self._stop_motion()

    def _s_align_blue_drum(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        bd = r.drums.blue_drum
        if bd.detected:
            self._drum_map.record_blue(bd.cx, bd.cy)
            yaw = self._aligner.compute_yaw(bd.error_x)
            surge = (n["approach_speed"]
                     if bd.area < self._cfg["blue_drum"]["drop_min_area"] else 0.0)
            self._set_motion(surge=surge, yaw=yaw)
            if (self._aligner.is_aligned(bd.error_x) and
                    bd.area >= self._cfg["blue_drum"]["drop_min_area"]):
                log.info(f"Blue drum aligned: area={bd.area:.0f} error_x={bd.error_x}")
                self._transition(State.DROP_BALL, "aligned + close enough")
        else:
            self._stop_motion()

    def _s_drop_ball(self, r: PipelineResult, age: float):
        self._stop_motion()
        if age < 0.1:
            self._ibus.set_ball_drop(True)
            log.info("BALL DROP ACTIVATED")
        if age >= STATE_CONFIGS[State.DROP_BALL].timeout_s * 0.8:
            self._ibus.set_ball_drop(False)
            self._transition(State.GENERATE_MAP, "ball dropped")

    def _s_generate_map(self, r: PipelineResult, age: float):
        self._stop_motion()
        if age < 0.2:
            try:
                summary = self._drum_map.finalize("logs/drum_map.png")
                log.info(f"Drum map generated:\n{summary}")
            except Exception as e:
                log.error(f"Drum map generation failed: {e}", exc_info=True)
                self._pub_error("MAP_FAIL", str(e), "warning")
        if age >= 2.5:
            self._transition(State.FIND_EXIT, "map done")

    def _s_find_exit(self, r: PipelineResult, age: float):
        n = self._cfg["navigation"]
        self._set_motion(surge=n["surge_speed"])
        if age >= STATE_CONFIGS[State.FIND_EXIT].timeout_s * 0.9:
            self._transition(State.MISSION_COMPLETE, "at exit")

    # ── Helpers ───────────────────────────────────────────────────────────

    def _avoid_flare(self, flare) -> None:
        sway = 0.45 if flare.side == "left" else -0.45
        log.debug(f"Avoiding flare on {flare.side}")
        self._set_motion(surge=0.12, sway=sway)

    def _set_motion(self, surge=0.0, heave=0.0, yaw=0.0, sway=0.0) -> None:
        self._last_surge = surge
        self._last_heave = heave
        self._last_yaw   = yaw
        self._last_sway  = sway
        self._ibus.set_motion(surge=surge, heave=heave, yaw=yaw, sway=sway)
        self._dead_reck.update(surge, yaw)
        if self._pub:
            try:
                self._pub.pub_motion(MotionMsg(
                    surge=surge, heave=heave, yaw=yaw, sway=sway,
                    armed=True, ts=time.monotonic()
                ))
            except Exception:
                pass

    def _stop_motion(self) -> None:
        self._set_motion()

    def _enter_state(self, state: State) -> None:
        now = time.monotonic()
        # Close out previous
        if self._history:
            self._history[-1].exited = now
        rec = StateRecord(state=state, entered=now)
        self._history.append(rec)

        # Update active detectors in vision pipeline
        sc = STATE_CONFIGS.get(state)
        if sc:
            self._vision.set_active_detectors(sc.active_detectors)

        self._prev_state = self._state
        self._state      = state
        self._state_start = now
        self._aligner.reset()
        self._stop_motion()

        log.info(f"━━ STATE: {state.name} ━━ ({sc.description if sc else ''})")

        if self._pub:
            try:
                self._pub.pub_state(StateMsg(
                    state=state.name,
                    prev=self._prev_state.name,
                    ts=now,
                    age_s=0.0
                ))
            except Exception:
                pass

    def _transition(self, new_state: State, reason: str = "") -> None:
        log.info(f"Transition: {self._state.name} → {new_state.name}  [{reason}]  "
                 f"(was in state {self._state_age():.1f}s)")
        self._enter_state(new_state)

    def _safe_stop(self, reason: str = "") -> None:
        log.critical(f"SAFE STOP triggered: {reason}")
        self._ibus.emergency_stop()
        self._enter_state(State.SAFE_STOP)

    def _state_age(self) -> float:
        return time.monotonic() - self._state_start

    def _pub_error(self, code: str, detail: str, severity: str) -> None:
        if self._pub:
            try:
                self._pub.pub_error(ErrorMsg(
                    module="autonomy_core",
                    severity=severity,
                    code=code,
                    detail=detail,
                    ts=time.monotonic()
                ))
            except Exception:
                pass

    def _shutdown(self) -> None:
        log.info("=" * 60)
        log.info(f"MISSION ENDED in state: {self._state.name}")
        log.info(f"Tick count: {self._tick_count}  Loop Hz: {self._loop_hz:.1f}")
        log.info("State history:")
        for rec in self._history:
            duration = (rec.exited - rec.entered) if rec.exited else (time.monotonic() - rec.entered)
            log.info(f"  {rec.state.name:<22} {duration:6.1f}s  {rec.reason}")
        log.info("=" * 60)
        self._stop_motion()
        self._ibus.disarm()
