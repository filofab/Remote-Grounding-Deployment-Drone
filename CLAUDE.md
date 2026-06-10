# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A Remote Grounding Deployment Drone: an unmanned aerial system that autonomously installs grounding/earthing cables on live high-voltage (AT) transmission lines. The drone runs on a Raspberry Pi companion computer connected to a MAVLink flight controller (via DroneKit) and an Arducam Time-of-Flight (ToF) depth camera, with YOLOv8 person detection as a safety check before payload release.

**Language note:** comments, console output, documentation, and commit messages are written in Italian. Follow that convention ("passo" = mission step, "sgancio" = payload release, "cavo" = the power cable, "a cavallo del cavo" = straddling the cable).

## Running the Code

There is no build system, package manifest, or test suite. Scripts are run directly:

```bash
cd Python/dk_autonomo_V5   # always cd into the version folder — imports are flat (from util import *)
python main.py
```

Dependencies (install manually): `dronekit`, `pymavlink`, `opencv-python`, `numpy`, `ultralytics` (YOLO scripts only), and the `ArducamDepthCamera` SDK (hardware-specific).

**Hardware-bound:** the flight code requires a real drone broadcasting MAVLink on `udpin:0.0.0.0:14550` (see `CONNECTION_STRING` in `config.py`) and an Arducam ToF camera on CSI. It cannot run in a headless/CI environment. YOLO demo scripts only need a video file and model weights.

## Repository Structure & Versioning Convention

The flight code lives in versioned snapshot folders — this is the central convention of the repo:

- `Python/dk_autonomo_V0` … `dk_autonomo_V5` — frozen version snapshots. **V5 is the latest**; older folders are kept as backups and must not be modified.
- `Python/dk_autonomo_TEST` — beta version for internal testing.
- `Python/__log_di_lavoro.md` — the version changelog. **Update this file (in Italian) whenever a new version folder is created**, describing what the version adds.

New development goes into the newest version folder (or a new `dk_autonomo_VN` folder copied from the latest, for milestone changes).

Other top-level folders:

- `Yolo/` — YOLOv8 person-detection pipeline, independent of the flight code.
- `Arducam/` — standalone ToF camera example (`Example.py`) and camera configuration files (`ConfigCamera.json`, `ConfigAlgorithm.json`, `register.json`). Copies of these JSONs appear in some version folders.

## Flight Code Architecture (dk_autonomo_V5)

Each version folder has the same five-file layout:

- `main.py` — entry point. Connects to the drone, arms and takes off, opens the ToF camera, then runs the main loop: grab depth frame → measure cable distances → dispatch to the active mission step → render OpenCV preview with status popups → handle keyboard input.
- `config.py` — **all tunable parameters live here** (ToF thresholds, yaw PID gain `GUADAGNO_YAW`, tolerances, takeoff altitude, MAVLink connection string, AUX servo channel/PWM for camera tilt). Never hardcode these values in the logic files.
- `util.py` — shared helpers: image flip/text overlay, ToF ROI distance measurement (`measure_distance_vertical`/`_horizontal` use a percentile over a confidence-filtered ROI), MAVLink commands (`condition_yaw`, `set_velocity_body`, `set_servo_pwm`), proportional yaw controllers, and cable-centering detection (`trova_y_cavo`, `cavo_al_centro`).
- `dk_passo1.py` — mission step 1: align perpendicular to the cable by equalizing left/right ToF distances (yaw control, must hold tolerance for 5 s), then a guided approach-to-40cm / hover / retreat-to-1m maneuver (its own internal state machine 0–3).
- `dk_passo2.py` — mission step 2: climb by `INCREMENTO_ALTEZZA`, tilt the ToF camera downward via AUX servo, advance until the cable is centered in frame, yaw 90° to straddle the cable, then `resta_sopra_il_cavo` keeps the drone stationed above it (re-checked every 5 s).

### Mission State Machine

`main.py` drives the mission with a `token` variable: `0` = manual/controlled flight, `1` = step 1.1 (alignment), `2` = step 1.2 (approach/retreat), `3` = step 2 (positioning over the cable), `4` = station-keeping over the cable awaiting release. Each step signals completion via `attesa_conferma*` flags, which show a confirmation popup; the operator advances with the keyboard.

Keyboard commands: `y` start/confirm next step, `r` reset to token 0, `s` pause, `l` land, `q` quit, `d` release payload (sgancio), `p` toggle simulated person detection (safety-check placeholder for the YOLO integration).

The operator-confirmation gates between steps are a deliberate safety design — do not remove them when modifying mission logic.

## YOLO Pipeline (Yolo/)

Numbered scripts form the training workflow:

- `1_TRAINING.py` — train YOLOv8 (`yolov8n.pt` base) on `dataset/data.yaml`.
- `2_VALIDAZIONE.py` — validate `runs/detect/yolo_custom/weights/best.pt`.
- `3_TEST.py` — single-image prediction test.
- `Frame_extractor.py` — extract frames from video for dataset building (`python Frame_extractor.py <video> <interval>`).
- `Yolo_V1.py` / `Yolo_V2.py` / `Yolo_V3.py` — versioned person-detection demos on video using the custom-trained model `colab_trained.pt` (class 0 = person); V3 is the latest. These set the `persona_detected` boolean that the flight code's `p` key currently simulates — the planned integration is feeding this flag into step 3 (pre-release safety check).
