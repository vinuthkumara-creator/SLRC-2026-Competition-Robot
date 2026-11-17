# SLRC-2026-Competition-Robot
The Sri Lankan Robotics Challenge (SLRC) is one of the country's largest annual robotics competitions, organized by the Electronic Club of the Department of Electronic and Telecommunication Engineering at the University of Moratuwa.

<p align="center">
  <img width="48%" alt="image" src="https://github.com/user-attachments/assets/59440a73-1d0d-4d36-9aa5-7bc6fe46fe36" />
  <img width="48%" height="1500" alt="image" src="https://github.com/user-attachments/assets/3e860d6f-9c48-4df6-bceb-54f04577208b" />
</p>
---

## Table of contents

- [About us](#about-us)
- [The competition](#the-competition)
- [Components](#components)
- [Mechanical design](#mechanical-design)
- [Firmware](#firmware)
- [Build and flash](#build-and-flash)
- [Repository structure](#repository-structure)
- [Lessons learned](#lessons-learned)
- [License](#license)

---

## About us

Team Members:

- Vinuth Kumarasinghe
- Sasanka Lochana
- Akash Jayasinghe
- Yevin Perera
- Tharusha Dambarage

Having been able to explore lot of new things in the field of electronics during the time I worked with another team for the Xbotix 2025 competition, this time in 2026, I have taken a step forward to build my own team, and participate in SLRC 2026. Although we could get over the elimination rounds, due to some beginner level issues and the academic workload we had to balance over the semester, we couldn't bring it to the level of success, we were exepecting. But there were lot of new things, we got to learn from experience.  

## The competition
The competition covered the following challenges:

- Line following
- Wall following
- Object detection
- Object grabbing
- April-Tag Identification
- Colour identification
  
The official rulebook is available here: [SLRC 2026 Rulebook](docs/rulebook/SLRC2026_rulebook.pdf)

## Components

| Part | Used for |
|---|---|
| Arduino Mega | Main controller |
| JGA25-370 motors with encoders | Drive and position feedback |
| Ultrasonic sensors | Object and wall detection |
| TCRT5000 8-channel IR array | Line following |
| 2 additional IR sensors | Junction / side detection |
| Stepper Motor | Grabber, feeder|

## Mechanical design

The feeder, grabber weren't designed by our own. An already designed 3D grabber is taken,    
<p align="center">
  <img width="48%" alt="image" src="https://github.com/user-attachments/assets/76f968e8-84d6-4f4a-994f-b9bcf9c3b402" />
</p>

## Firmware

All code is written using the Arduino IDE.

**Competition sketches** (`firmware/competition/`)

| Sketch | Purpose |
|---|---|
| `penatronix_codes` | Final Elemination Round code, with the task selected using DIP switches |
| `wall_following` | Following a dotted line |
| `PID_Line Follow` | Maze solving using the right-hand rule |
| `April tag detection` | Junction detection with box grabbing |

## Build and flash

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Copy the folders inside `firmware/libraries/` into your Arduino `libraries` folder (usually `Documents/Arduino/libraries`).
3. Install the other required libraries listed in [`firmware/libraries/README.md`](firmware/libraries/README.md).
4. Open a sketch from `firmware/competition/` or `firmware/tests/`.
5. Select your board, choose the port and upload.

## Repository structure

```
.
├── docs/                 Pinout and documentation
├── firmware/
│   ├── competition/      Sketches used in the competition
│   ├── tests/            Small sketches for testing each subsystem
│   └── libraries/        Custom and required libraries
├── hardware/
│   └── mechanical/       3D models and design images
└── media/
    └── photos/           Photos of the robot
```

## Lessons learned

- Starting with a first competition taught us how much time goes into testing and tuning, not just writing code.
- The most important thing which we learnt from experience is that, the impairment in between our theoretical perspective and real physical behavior, also that most of the work should revolve around testing and fine tuning, not just understanding theories and applying them.  

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for details.
