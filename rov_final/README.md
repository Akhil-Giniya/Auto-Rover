# Autonomous Underwater ROV — Raspberry Pi Autonomy System
### Robofest 5.0 Competition Build

---

## What This Code Does — In Plain English

This is the **brain software** that runs on the Raspberry Pi inside the submarine.

The submarine has two computers inside it:
- **ESP32** — the low-level controller. It reads the motors, sensors, and keeps the sub stable. **We do not touch this code. It is already written and working.**
- **Raspberry Pi 4** — this is where this code runs. It watches the camera, decides where to go, and sends commands to the ESP32.

The Raspberry Pi talks to the ESP32 through a cable (UART serial), sending a standard RC signal called **iBus** — the same signal a radio controller would send. The ESP32 thinks it is receiving pilot commands, but it is actually receiving commands from our code.

---

## What the Code Can Do

### Vision (Camera)
- Reads the camera at **640×480, up to 30 frames per second**
- **Detects the orange flare** obstacle using colour detection and automatically steers around it
- **Detects the qualification gate** (metal pipe frame) using edge detection and finds its centre
- **Detects the green mat** target zone on the pool floor
- **Detects red drums and the blue drum** by colour, calculates where each one is in the frame

### Navigation
- **Aligns to any target** using a control algorithm (IBVS PID) — if the gate is 50 pixels to the right, it calculates exactly how much to turn
- **U-turn** — rotates 180 degrees using timed yaw at a calibrated rate
- **Dead reckoning** — tracks approximate X/Y position using speed and heading over time
- **Flare avoidance** — if the orange flare appears on the left, slide right; if on the right, slide left

### Mission (State Machine)
Runs the complete 12-step competition mission automatically:

| Step | What Happens |
|------|-------------|
| 1 | Drive forward through the start gate |
| 2 | Search for and align to the qualification gate |
| 3 | Drive through the gate forward |
| 4 | Perform a 180° U-turn |
| 5 | Drive back through the gate in reverse |
| 6 | Find the green mat on the pool floor |
| 7 | Detect all drums (3 red + 1 blue) |
| 8 | Align precisely to the blue drum |
| 9 | Drop the ball into the blue drum |
| 10 | Generate and save a 2D map of where all drums are |
| 11 | Navigate to the exit gate |
| 12 | Mission complete |

Every step has a **hard time limit** — if it takes too long, the sub moves on instead of getting stuck forever.

### Safety and Error Handling
- **Watchdog** — constantly checks that the camera and motor link are alive. If either dies, it sends an emergency stop to the ESP32 automatically
- **Auto-reconnect** — if the serial cable to the ESP32 has a glitch, the code reconnects automatically (up to 10 attempts)
- **Per-frame error isolation** — if the colour detector crashes on one frame, only that frame is skipped; the rest of the system keeps running
- **Slew rate limiter** — never sends a sudden jump to the motors; all commands are smoothly ramped
- **Emergency stop** — any part of the code can trigger an immediate disarm and zero all motors

### Logging and Telemetry
- Every module writes its own rotating log file in `logs/`
- A machine-readable `telemetry.jsonl` file records every event for post-run analysis
- A `CRITICAL.log` file records only the most serious errors and never gets deleted
- After any run, use `python tools/view_logs.py` to see a clean summary of what happened

---

## Hardware This Runs On

| Component | Details |
|-----------|---------|
| Computer | Raspberry Pi 4 Model B |
| Camera | Raspberry Pi Camera Module 2 |
| Connection to ESP32 | UART serial (TX pin → ESP32 GPIO16) |
| ESP32 sensors | LSM9DS1 IMU, MS5837 depth sensor |
| Thrusters | 4× brushless motors via ESC |
| Servos | Front and rear tilt + ball drop |

**The ESP32 firmware is NOT modified.** This code only generates iBus packets that the ESP32 receives as if from a radio controller.

---

## How to Set Up (First Time Only)

### Step 1 — Copy the code onto the Raspberry Pi
```bash
# On your laptop, copy the folder to the Pi over SSH
scp -r rov_final/ pi@<your-pi-ip>:~/rov_final
```

### Step 2 — Run the installer
```bash
cd ~/rov_final
bash install.sh
```
This installs all required Python packages and enables the UART serial port.
**Reboot the Raspberry Pi after this step.**

### Step 3 — Wire the UART
Connect a wire from **Raspberry Pi TX (GPIO 14, Pin 8)** to **ESP32 GPIO16**.
Connect **GND to GND** between both boards.

---

## How to Run

### Normal competition run
```bash
bash run.sh
```
This arms the sub and runs the full autonomous mission.

### Practice / debug run (no motors, shows camera window)
```bash
bash debug.sh
```
Use this to test the vision and logic on a bench without activating any motors.

### Run on specific serial port
```bash
bash run.sh --port /dev/ttyS0
```

### Keyboard shortcut during any run
Press `Ctrl+C` to trigger a graceful shutdown — it disarms the sub and closes everything safely.

---

## Before Every Competition Session

**1. Tune the colours in the actual pool under competition lighting:**
```bash
python tools/tune_hsv.py --target blue_drum
python tools/tune_hsv.py --target orange_flare
python tools/tune_hsv.py --target green_mat
```
Move the sliders until only the target object is highlighted. Press `s` to save. Press `q` to quit.

**2. Bench test the motor link (props off or out of water):**
```bash
python tools/test_ibus.py
```
This sweeps all motors and servos slowly. Watch the ESP32 serial output — you should see `armed=true` and the commands changing.

**3. Run dry-run in the pool to check vision:**
```bash
bash debug.sh
```
Confirm the camera sees the gate, drums, and mat correctly before a real run.

---

## After Every Run — Check the Logs

```bash
python tools/view_logs.py
```

This shows:
- How many state transitions happened
- Whether any errors or warnings occurred
- The full timeline of what the sub did

Detailed logs are saved in `logs/`. The file `logs/drum_map.png` shows a 2D picture of where the drums were detected.

---

## How to Run the Tests (No Hardware Needed)

```bash
bash test.sh
```

This runs **44 automated tests** covering every module: iBus packet building, PID math, vision detectors on synthetic images, watchdog behaviour, and config validation. All tests run on any laptop without a Raspberry Pi or camera.

---

## Tuning Values (What to Adjust at the Competition)

All tunable values are in **`config.json`**. The ones you are most likely to need:

| Setting | Where | What it does |
|---------|-------|-------------|
| `surge_speed` | `navigation` | How fast the sub drives forward (0.0–1.0) |
| `approach_speed` | `navigation` | Slower speed when aligning to a target |
| `uturn_yaw_speed` | `navigation` | Yaw rate during U-turn |
| `drop_min_area` | `blue_drum` | How close to blue drum before dropping (pixels²) |
| `center_tolerance_px` | `alignment` | How many pixels off-centre counts as "aligned" |
| HSV values | all detectors | Colour ranges — tune with `tune_hsv.py` |

---

## Final Folder Structure

```
rov_final/                          ← ROOT — everything lives here
│
├── README.md                       ← THIS FILE — start here
├── main.py                         ← ENTRY POINT — run this to start the mission
├── rov_logger.py                   ← Logging system used by every module
├── config.json                     ← All tunable values (speeds, colours, PID gains)
├── requirements.txt                ← Python packages needed
│
├── install.sh                      ← One-shot installer (run once on the Pi)
├── run.sh                          ← Start a real mission
├── debug.sh                        ← Dry-run with camera display, no motors
├── test.sh                         ← Run all 44 unit tests
│
├── hw_interface/                   ← Talks to the ESP32
│   └── ibus_interface.py           ← Builds and sends iBus packets over UART
│
├── vision/                         ← Eyes of the submarine
│   └── pipeline.py                 ← Camera capture + flare/gate/mat/drum detection
│
├── autonomy_core/                  ← Brain of the submarine
│   └── state_machine.py            ← 12-state mission FSM
│
├── navigation/                     ← Motion calculations
│   └── alignment.py                ← PID controller, IBVS alignment, dead reckoning
│
├── mapping/                        ← Records where drums are
│   └── drum_map.py                 ← Drum position tracker + 2D map generator
│
├── telemetry/                      ← Health monitoring
│   ├── watchdog.py                 ← Monitors all modules, fires emergency stop
│   └── bus.py                      ← ZeroMQ pub/sub message bus between modules
│
├── tests/                          ← Automated tests (run without hardware)
│   └── test_all.py                 ← 44 unit tests
│
├── tools/                          ← Helper tools for setup and debugging
│   ├── tune_hsv.py                 ← Interactive colour tuner (use before competition)
│   ├── test_ibus.py                ← Motor link bench test (use before pool)
│   └── view_logs.py                ← Read and summarise mission logs
│
└── logs/                           ← All log files land here (auto-created)
    ├── autonomy_core_YYYY-MM-DD.log
    ├── vision_YYYY-MM-DD.log
    ├── hw_interface_YYYY-MM-DD.log
    ├── telemetry.jsonl             ← Machine-readable full event log
    ├── CRITICAL.log                ← Serious errors only, never deleted
    └── drum_map.png                ← 2D map of drum positions from last run
```

---

## Quick Reference Card

| I want to... | Command |
|-------------|---------|
| Run the mission | `bash run.sh` |
| Test without motors | `bash debug.sh` |
| Tune colours for the pool | `python tools/tune_hsv.py --target blue_drum` |
| Test motor link (props off) | `python tools/test_ibus.py` |
| Run all tests | `bash test.sh` |
| See what happened last run | `python tools/view_logs.py` |
| Change mission timing | Edit `config.json` → `navigation` section |
| Change colour ranges | Run `tune_hsv.py` or edit `config.json` directly |

---

## Important Notes

- **Never run `bash run.sh` with the sub in hand.** Always place it in water or remove propellers first.
- **The ESP32 will not move until it receives armed iBus.** The code arms automatically after a 2-second delay.
- **If the sub behaves erratically**, press `Ctrl+C` immediately — graceful shutdown disarms within 0.5 seconds.
- **Colour tuning is the most important pre-competition task.** Pool lighting changes everything. Always re-tune on the day.
- **Log files grow over time.** They rotate automatically at 5 MB per file. Old logs are kept for 5 rotations.
