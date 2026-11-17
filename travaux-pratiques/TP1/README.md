# TP1 - Getting started with the haptic paddle

First lab of the course: basic firmware for the 1-DoF haptic paddle (STM32F407).
`haptic_controller.c` reads the Hall sensor and the incremental encoder and computes the paddle angle.
It exposes `spring_k` and `zero_angle` to the PC interface; the virtual spring torque is prepared but commented out (disabled) in this snapshot, so no torque is applied.
It also estimates velocity and acceleration (1- and 2-sample differences); a first-order low-pass filter is applied only to the 1-sample velocity and acceleration.

Only the controller is kept here; the rest of the firmware is the course skeleton (see the repository root).
