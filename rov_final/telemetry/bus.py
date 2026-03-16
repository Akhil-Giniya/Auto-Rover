"""
telemetry/bus.py
─────────────────────────────────────────────────────────────────────────────
ZeroMQ-based telemetry bus.

Architecture
────────────
                ┌──────────────┐
  Publishers    │  XPUB/XSUB   │   Subscribers
  (modules)  ──▶│   Broker     │──▶ (dashboard, logger, other modules)
                └──────────────┘
                   port 5555/5556

Every module can publish and subscribe without knowing about each other.
The broker is started once by main.py.

Topics (string prefix before the first ':')
──────────────────────────────────────────
  state:          current FSM state + timestamp
  vision:         detection results per frame
  motion:         iBus channel values being sent
  health:         watchdog heartbeats + error counts
  telemetry:      depth, heading, camera fps, loop timing
  error:          structured error events

Serialization:  MessagePack (fast binary, schema-free)
"""

import threading
import time
import json
from dataclasses import dataclass, asdict
from typing import Callable, Optional

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    import msgpack
    MSGPACK_AVAILABLE = True
except ImportError:
    MSGPACK_AVAILABLE = False

from rov_logger import get_logger

log = get_logger("telemetry.bus")

PUB_PORT  = 5555
SUB_PORT  = 5556


# ── Data contracts ─────────────────────────────────────────────────────────

@dataclass
class StateMsg:
    state:     str
    prev:      str
    ts:        float
    age_s:     float

@dataclass
class VisionMsg:
    flare_detected:  bool
    gate_detected:   bool
    gate_error_x:    int
    blue_drum:       bool
    blue_area:       float
    blue_error_x:    int
    red_count:       int
    ts:              float

@dataclass
class MotionMsg:
    surge:   float
    heave:   float
    yaw:     float
    sway:    float
    armed:   bool
    ts:      float

@dataclass
class HealthMsg:
    module:        str
    status:        str       # "ok" | "degraded" | "error" | "dead"
    error_count:   int
    uptime_s:      float
    ts:            float

@dataclass
class TelemetryMsg:
    camera_fps:    float
    loop_hz:       float
    ibus_age_ms:   float
    ts:            float

@dataclass
class ErrorMsg:
    module:    str
    severity:  str           # "warning" | "error" | "critical"
    code:      str           # machine-readable tag e.g. "CAMERA_TIMEOUT"
    detail:    str
    ts:        float


def _pack(obj) -> bytes:
    d = asdict(obj)
    if MSGPACK_AVAILABLE:
        return msgpack.packb(d, use_bin_type=True)
    return json.dumps(d).encode()

def _unpack(data: bytes) -> dict:
    if MSGPACK_AVAILABLE:
        return msgpack.unpackb(data, raw=False)
    return json.loads(data.decode())


# ── Broker (run once in a daemon thread) ───────────────────────────────────

class TelemetryBroker:
    """XPUB/XSUB proxy — started once by main.py."""

    def __init__(self):
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        if not ZMQ_AVAILABLE:
            log.warning("ZeroMQ not available — telemetry bus disabled")
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="zmq-broker")
        self._thread.start()
        log.info(f"Telemetry broker started  pub={PUB_PORT} sub={SUB_PORT}")

    def _run(self):
        ctx = zmq.Context()
        xpub = ctx.socket(zmq.XPUB)
        xsub = ctx.socket(zmq.XSUB)
        xpub.bind(f"tcp://*:{SUB_PORT}")
        xsub.bind(f"tcp://*:{PUB_PORT}")
        try:
            zmq.proxy(xsub, xpub)
        except zmq.ZMQError:
            pass
        finally:
            xpub.close()
            xsub.close()
            ctx.term()


# ── Publisher ─────────────────────────────────────────────────────────────

class TelemetryPublisher:
    """One per module — thread-safe publish."""

    def __init__(self, module: str):
        self._module = module
        self._sock = None
        self._lock = threading.Lock()
        if ZMQ_AVAILABLE:
            ctx = zmq.Context.instance()
            self._sock = ctx.socket(zmq.PUB)
            self._sock.connect(f"tcp://localhost:{PUB_PORT}")
            time.sleep(0.05)   # ZMQ slow-joiner

    def publish(self, topic: str, msg) -> None:
        if self._sock is None:
            return
        try:
            with self._lock:
                payload = _pack(msg)
                self._sock.send_multipart([topic.encode(), payload])
        except Exception as e:
            log.warning(f"[{self._module}] publish error: {e}")

    def pub_state(self, msg: StateMsg):      self.publish("state",     msg)
    def pub_vision(self, msg: VisionMsg):    self.publish("vision",    msg)
    def pub_motion(self, msg: MotionMsg):    self.publish("motion",    msg)
    def pub_health(self, msg: HealthMsg):    self.publish("health",    msg)
    def pub_telemetry(self, msg: TelemetryMsg): self.publish("telemetry", msg)
    def pub_error(self, msg: ErrorMsg):      self.publish("error",     msg)


# ── Subscriber ────────────────────────────────────────────────────────────

class TelemetrySubscriber:
    """Subscribe to one or more topics, callback on message."""

    def __init__(self, topics: list[str], callback: Callable[[str, dict], None]):
        self._topics   = topics
        self._callback = callback
        self._thread   = None
        self._running  = False
        self._sock     = None

    def start(self):
        if not ZMQ_AVAILABLE:
            return
        ctx = zmq.Context.instance()
        self._sock = ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://localhost:{SUB_PORT}")
        for t in self._topics:
            self._sock.setsockopt_string(zmq.SUBSCRIBE, t)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            try:
                if self._sock.poll(timeout=200):
                    parts = self._sock.recv_multipart()
                    if len(parts) == 2:
                        topic = parts[0].decode()
                        data  = _unpack(parts[1])
                        self._callback(topic, data)
            except Exception as e:
                log.warning(f"Subscriber error: {e}")

    def stop(self):
        self._running = False
