# Arm and Gripper

This section explains how we built the object-grabbing mechanism for the SLRC 2026 robot. It has two parts: a **stepper motor** that moves the arm and gripper horizontally, and **two MG90 servo motors** , one for rotation of the arm vertically and other one for openning and closing the gripper. Everything is controlled by the Arduino Mega.

<img width="50%" alt="image" src="https://github.com/user-attachments/assets/36bd5da2-a8ce-4eb6-8035-f77978d6f80e" />

---

## Overview

| Part | Motor | What it does |
|---|---|---|
| Horizontal slide | Stepper motor (Nema 17 Stepper) with A4988 Stepper driver | Moves the arm and gripper sideways along a rail |
| Gripper | 2 x MG90 servo motors | one rotating arm vertically and other one Opens and closes the gripper fingers |

We split the job between two kinds of motor because each suits a different task. The stepper moves in fixed steps, so we could place the gripper at an exact position without a position sensor. The MG90 servos are small and light, and they turn to an angle we command directly, which is what a gripper at the end of an arm needs.

---

## Horizontal movement (stepper motor)

### Why we used a stepper

The robot needs to line the gripper up with an object accurately. Instead of repositioning the whole robot for every small correction, we let the stepper slide the arm and gripper sideways. Because a stepper moves a known amount per step, the distance travelled can be controlled just by counting steps, and it holds its position firmly when stopped.

### How it works

1. The Arduino Mega sends step pulses to the stepper driver. Each pulse moves the motor one small step.
2. A direction signal tells the driver which way to turn, which decides whether the assembly moves left or right.
3. The driver supplies the current for the motor coils, since the Arduino cannot drive the motor directly.
4. The motor moves the carriage, which holds the arm and gripper, along a rail using a [lead screw / belt].

### Homing

A stepper does not know where it is when the robot is switched on. At start-up the carriage moves slowly toward one end of the rail until it presses a limit switch. That point becomes zero, and every later movement is measured from it.

### Mechanical notes

- The carriage is supported by [the rail / guide rods], so the [lead screw / belt] only needs to push the carriage and does not carry the weight.
- We kept the moving parts as light as we could, because extra weight means the motor skips steps at higher speeds.
- The servo wires have slack so they are not pulled when the carriage moves.

---

## Gripper (MG90 servos)

### How it works

Each MG90 servo turns to an angle set by the control signal from the Arduino, across about 180 degrees. We use two servos, one for each finger. They face each other, so their angles are mirrored: while one servo turns to an angle, the other turns to 180 minus that angle, and the two fingers move together.

### Mechanical notes

- The closed angle is chosen so the gripper holds the object firmly without the servos fully stalling against it. A stalled servo draws a lot of current, heats up and can damage its gears.
- [Rubber / foam] on the fingertips improves the grip.
---

## Wiring and power

- **Stepper driver:** step, direction and (optionally) enable lines go to the Arduino Mega. The motor has its own supply of 12V.
- **Limit switch:** connected between an Arduino input pin and ground, using the internal pull-up resistor.
- **Servos:** each servo signal wire goes to its own Arduino pin. The servos are powered from a separate [5 to 6 V] supply, not the Arduino 5 V pin, because servos draw current spikes that can reset the Arduino.
- **Common ground:** the Arduino, driver, servo supply and motor supply all share a ground.

---

## How it all worked together

1. The robot starts with the gripper open and the arm homed.
2. It drives up to the object using the ultrasonic sensors and encoders.
3. The stepper slides the arm and gripper into line with the object.
4. The servos close the gripper to grab it.
5. The robot carries the object to the drop point, where the gripper opens to release it.

---

## What we learned

- Separate small test sketches for the arm, gripper and wheels saved us a lot of time when something stopped working.
- Setting the stepper speed and acceleration too high made it skip steps, so we had to reduce them until the movement was reliable.
- Powering the servos from their own supply avoided random resets.
