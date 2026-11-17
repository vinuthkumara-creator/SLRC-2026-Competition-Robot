# Drive System: JGA25-370 Encoder Motors and L298N Driver
 
This section explains how we drove the SLRC 2026 robot accurately along the line. We used two **JGA25-370 geared DC motors with encoders** for the wheels, and an **L298N motor driver** to control them from the Arduino Mega.
 |<img width="50%" alt="image" src="https://github.com/user-attachments/assets/8039ce52-898d-42d2-887c-8ad3fde778b1" />|<img width="50%" alt="image" src="https://github.com/user-attachments/assets/d12bce83-93b0-4037-acf8-f8ac53576767" />|
 
---
 
## Overview
 
| Part | Role |
|---|---|
| JGA25-370 motors with encoders | Drive the wheels and report how far each wheel has turned |
| L298N motor driver | Sets the direction and speed of each motor |
| TCRT5000 8-channel IR array | Tells the robot where the line is |
| Arduino Mega | Reads the sensors and encoders and controls the driver |
 
The IR array tells the robot *where it should go*, the L298N *makes the wheels move*, and the encoders *confirm how far the wheels actually moved*.
 
---
 
## JGA25-370 encoder motors
 
### What they are
 
The JGA25-370 is a small DC motor with a gearbox, so it gives good torque at a usable wheel speed. It also has a built-in magnetic encoder on the back of the motor shaft. As the shaft turns, the encoder produces two pulse signals (channels A and B) that are slightly out of phase with each other.
 
### Why we used encoders
 
A plain DC motor spins at a speed that changes with battery voltage and load, so two motors given the same command will rarely turn at exactly the same speed. That makes a robot drift off a straight path. Encoders solved this for us in two ways:
 
- **Matching the wheels:** by counting the pulses from each wheel, the Arduino knew how fast each wheel was really turning and could correct any difference, so the robot travelled straighter.
- **Measuring distance and angle:** counting pulses lets the robot travel an exact distance or turn by an exact angle, instead of relying on timing.
### How the counting works
 
- The Arduino counts the pulses from each encoder using interrupts, so no pulse is missed even while the rest of the code is running.
- The second channel tells the Arduino which way the wheel is turning, so the count goes up when moving forward and down when reversing.
- The wheel's distance travelled comes from the number of pulses counted, the number of pulses per wheel revolution ([counts per revolution], which depends on the motor's encoder and gear ratio), and the wheel circumference ([wheel diameter]).
- For turning, the robot uses the distance between the two wheels ([track width]) to work out how far each wheel must travel to rotate the robot by a given angle.
We used this in the `go_specific_distance` and `rotate_to_deg` sketches, and tested wheel rotation separately in `wheel_rotate_func`.
 
---
 
## L298N motor driver
 
### What it does
 
The Arduino pins can only supply a few milliamps, far too little to run the motors. The L298N is a dual H-bridge driver, which means it can switch a much larger current from the battery to two motors, and it can reverse the current direction in each motor. One L298N controls both drive motors.
 
### How we controlled it
 
Each motor uses three control connections from the Arduino:
 
| Connection | Purpose |
|---|---|
| Two direction inputs (IN1/IN2 for one motor, IN3/IN4 for the other) | Set the motor to spin forward, backward or stop |
| An enable pin (ENA / ENB), driven with PWM | Sets the motor speed |
 
Speed control works by PWM (pulse width modulation). The Arduino switches the enable pin on and off very quickly, and a larger share of "on" time makes the motor run faster.
 
### Power notes
 
- The L298N loses roughly 1.4 to 2 V inside the driver, so the motors receive a little less than the battery voltage. We chose the battery voltage with this in mind ([battery voltage]).
- The Arduino, driver and battery all share a common ground.
---
 
## Putting it together on the line
 
1. The TCRT5000 IR array reads the line and gives the robot a measure of how far it is to the left or right of the line.
2. The Arduino turns that error into a correction using PID control. The tuning values were adjusted using the `tuning_bot` sketch.
3. The correction makes one wheel faster and the other slower, and the L298N applies those speeds through PWM.
4. The encoders report how fast each wheel is actually moving, so the Arduino can keep both wheels in step and drive straight on the straight sections.
5. At junctions and turns, the encoders let the robot move a set distance forward or rotate to a set angle before it continues.
---
 
## What we learned
 
- Encoder counts made turns and distances far more repeatable than timing-based movement.
- The two motors never behaved identically, so we needed to tune each side separately.
- The L298N voltage drop and heat were things we had to plan around.
