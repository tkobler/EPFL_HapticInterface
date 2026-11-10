# TP1 - Getting started with the haptic paddle

First lab of the course: basic firmware for the 1-DoF haptic paddle (STM32F407).
`haptic_controller.c` reads the Hall sensor and the incremental encoder, converts them to a paddle angle,
and implements a virtual spring (stiffness and zero angle tunable from the PC interface).
It also estimates velocity and acceleration (1- and 2-sample differences) and low-pass filters them.

Only the controller is kept here; the rest of the firmware is the course skeleton (see the repository root).
