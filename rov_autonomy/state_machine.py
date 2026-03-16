"""
state_machine.py
12-state autonomous mission FSM for the ROV.

States exactly match the PRD:
  1  ENTER_ARENA       — move forward through start gate
  2  SEARCH_GATE       — find and align to qualification gate
  3  PASS_GATE_FWD     — drive through gate forward
  4  U_TURN            — 180° turn using timed yaw
  5  PASS_GATE_REV     — pass back through gate in reverse
  6  SEARCH_GREEN_MAT  — find the green target mat
  7  DETECT_DRUMS      — find and classify red/blue drums
  8  ALIGN_BLUE_DRUM   — IBVS yaw align to blue drum center
  9  DROP_BALL         — trigger servo ball release
  10 GENERATE_MAP      — finalize and save drum position map
  11 FIND_EXIT         — navigate to finish gate
  12 MISSION_COMPLETE  — stop all motion

Design rules (from ESP32 code analysis):
  - RPi sends iBus at 50 Hz; if it drops out >500 ms ESP32 failsafes
  - Object detection is STATE FILTERED — only the relevant detector runs
  - U-turn uses timed yaw (no heading feedback from ESP32 needed)
  - depth hold: send CH3 = 1500 and rely on ESP32 PITCH_STAB depth PID
"""

import time
import logging
from enum import IntEnum, auto

from control.ibus_interface import IBusInterface
from vision.camera import Camera
from vision.detectors import (
    detect_flare, detect_gate, detect_green_mat, detect_drums, draw_debug
)
from navigation.alignment import IBVSAligner
from mapping.drum_map import DrumMap

import cv2

log = logging.getLogger("StateMachine")


class State(IntEnum):
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


# ──────────────────────────────────────────────────────────────────────────────
# Timing constants (seconds) — adjust during pool testing
# ──────────────────────────────────────────────────────────────────────────────
T_ENTER_ARENA      = 3.0    # drive forward through start gate
T_PASS_GATE_FWD    = 2.5    # drive through gate once aligned
T_U_TURN           = 4.5    # yaw at fixed rate for 180° (tune: rate × time ≈ 180°)
T_PASS_GATE_REV    = 2.5    # drive through gate in reverse
T_SEARCH_TIMEOUT   = 15.0   # give up searching and move on
T_DROP_BALL        = 1.5    # hold servo open duration
T_FIND_EXIT        = 5.0    # drive toward exit gate


class MissionStateMachine:

    def __init__(self, ibus: IBusInterface, camera: Camera, cfg: dict,
                 debug_display: bool = False):
        self._ibus   = ibus
        self._camera = camera
        self._cfg    = cfg
        self._debug  = debug_display
        self._aligner = IBVSAligner(cfg)
        self._drum_map = DrumMap()

        self._state = State.ENTER_ARENA
        self._state_start = time.monotonic()
        self._running = False

        # Ball drop: uses a dedicated servo channel
        # Map to CH5 (servo front, idx 4) — 1000=closed, 2000=open
        self._BALL_DROP_CH = 4
        self._BALL_CLOSED  = 1000
        self._BALL_OPEN    = 2000

    # ──────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────
    def run(self) -> None:
        """Blocking mission loop. Call after ibus.start()."""
        self._running = True
        log.info("Mission started")
        self._enter_state(State.ENTER_ARENA)

        while self._running and self._state != State.MISSION_COMPLETE:
            t_start = time.monotonic()

            frame = self._camera.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            self._tick(frame)

            # Debug display
            if self._debug and frame is not None:
                cv2.imshow("ROV debug", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Pace loop to ~25 Hz
            elapsed = time.monotonic() - t_start
            if elapsed < 0.04:
                time.sleep(0.04 - elapsed)

        self._shutdown()

    def stop(self) -> None:
        self._running = False

    # ──────────────────────────────────────────────────────────────────
    # State machine tick
    # ──────────────────────────────────────────────────────────────────
    def _tick(self, frame) -> None:
        s = self._state
        age = self._state_age()

        # ── Always check for flare in states 1–3 ──
        if s in (State.ENTER_ARENA, State.SEARCH_GATE, State.PASS_GATE_FWD):
            flare = detect_flare(frame, self._cfg)
            if flare.detected:
                self._avoid_flare(flare)
                return

        # ── State handlers ──
        if s == State.ENTER_ARENA:
            self._ibus.set_motion(surge=self._cfg["navigation"]["surge_speed"])
            if age >= T_ENTER_ARENA:
                self._transition(State.SEARCH_GATE)

        elif s == State.SEARCH_GATE:
            gate = detect_gate(frame, self._cfg)
            if gate.detected:
                yaw = self._aligner.compute_yaw(gate.error_x)
                self._ibus.set_motion(surge=self._cfg["navigation"]["approach_speed"],
                                      yaw=yaw)
                if self._aligner.is_aligned(gate.error_x):
                    self._transition(State.PASS_GATE_FWD)
            else:
                # Slow forward search
                self._ibus.set_motion(surge=0.15)
            if age >= T_SEARCH_TIMEOUT:
                log.warning("Gate search timeout — advancing anyway")
                self._transition(State.PASS_GATE_FWD)

        elif s == State.PASS_GATE_FWD:
            self._ibus.set_motion(surge=self._cfg["navigation"]["surge_speed"])
            if age >= T_PASS_GATE_FWD:
                self._transition(State.U_TURN)

        elif s == State.U_TURN:
            # Fixed yaw rate for calibrated time → ≈ 180°
            self._ibus.set_motion(yaw=self._cfg["navigation"]["uturn_yaw_speed"])
            if age >= T_U_TURN:
                self._transition(State.PASS_GATE_REV)

        elif s == State.PASS_GATE_REV:
            gate = detect_gate(frame, self._cfg)
            if gate.detected:
                yaw = self._aligner.compute_yaw(gate.error_x)
                self._ibus.set_motion(surge=self._cfg["navigation"]["surge_speed"],
                                      yaw=yaw)
            else:
                self._ibus.set_motion(surge=self._cfg["navigation"]["surge_speed"])
            if age >= T_PASS_GATE_REV:
                self._transition(State.SEARCH_GREEN_MAT)

        elif s == State.SEARCH_GREEN_MAT:
            mat = detect_green_mat(frame, self._cfg)
            if mat.detected:
                yaw = self._aligner.compute_yaw(mat.cx - 320)
                self._ibus.set_motion(surge=self._cfg["navigation"]["approach_speed"],
                                      yaw=yaw)
                if mat.area > self._cfg["green_mat"]["min_area"] * 4:
                    self._transition(State.DETECT_DRUMS)
            else:
                self._ibus.set_motion(surge=0.15)
            if age >= T_SEARCH_TIMEOUT:
                log.warning("Green mat timeout — advancing to drum detect")
                self._transition(State.DETECT_DRUMS)

        elif s == State.DETECT_DRUMS:
            drums = detect_drums(frame, self._cfg)
            # Record all drum positions for the map
            for d in drums.red_drums:
                self._drum_map.record_red(d.cx, d.cy)
            if drums.blue_drum.detected:
                self._drum_map.record_blue(drums.blue_drum.cx, drums.blue_drum.cy)
                self._transition(State.ALIGN_BLUE_DRUM)
            else:
                self._ibus.set_motion(surge=0.0)  # hover while scanning
            if age >= T_SEARCH_TIMEOUT:
                log.warning("Drum detect timeout")
                self._transition(State.ALIGN_BLUE_DRUM)

        elif s == State.ALIGN_BLUE_DRUM:
            drums = detect_drums(frame, self._cfg)
            if drums.blue_drum.detected:
                self._drum_map.record_blue(drums.blue_drum.cx, drums.blue_drum.cy)
                yaw = self._aligner.compute_yaw(drums.blue_drum.error_x)
                # Approach slowly until drum appears large enough
                surge = (self._cfg["navigation"]["approach_speed"]
                         if drums.blue_drum.area < self._cfg["blue_drum"]["drop_min_area"]
                         else 0.0)
                self._ibus.set_motion(surge=surge, yaw=yaw)

                aligned = self._aligner.is_aligned(drums.blue_drum.error_x)
                large_enough = drums.blue_drum.area >= self._cfg["blue_drum"]["drop_min_area"]
                if aligned and large_enough:
                    self._transition(State.DROP_BALL)
            else:
                self._ibus.stop_motion()
            if age >= T_SEARCH_TIMEOUT:
                self._transition(State.DROP_BALL)

        elif s == State.DROP_BALL:
            self._ibus.stop_motion()
            # Open servo immediately on state entry
            if age < 0.1:
                self._ibus.set_channel(self._BALL_DROP_CH, self._BALL_OPEN)
                log.info("BALL DROPPED")
            if age >= T_DROP_BALL:
                self._ibus.set_channel(self._BALL_DROP_CH, self._BALL_CLOSED)
                self._transition(State.GENERATE_MAP)

        elif s == State.GENERATE_MAP:
            self._ibus.stop_motion()
            if age < 0.1:
                self._drum_map.finalize("drum_map.png")
            if age >= 2.0:
                self._transition(State.FIND_EXIT)

        elif s == State.FIND_EXIT:
            self._ibus.set_motion(surge=self._cfg["navigation"]["surge_speed"])
            if age >= T_FIND_EXIT:
                self._transition(State.MISSION_COMPLETE)

    # ──────────────────────────────────────────────────────────────────
    # Flare avoidance (inline, not a full state)
    # ──────────────────────────────────────────────────────────────────
    def _avoid_flare(self, flare) -> None:
        """Dodge left or right based on which side flare is on."""
        sway = 0.4 if flare.side == "left" else -0.4
        self._ibus.set_motion(surge=0.15, sway=sway)

    # ──────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────
    def _enter_state(self, state: State) -> None:
        log.info(f"→ {state.name}")
        self._state = state
        self._state_start = time.monotonic()
        self._aligner.reset()
        self._ibus.stop_motion()

    def _transition(self, new_state: State) -> None:
        self._enter_state(new_state)

    def _state_age(self) -> float:
        return time.monotonic() - self._state_start

    def _shutdown(self) -> None:
        log.info("Mission complete — disarming")
        self._ibus.stop_motion()
        time.sleep(0.5)
        self._ibus.disarm()
        if self._debug:
            cv2.destroyAllWindows()
