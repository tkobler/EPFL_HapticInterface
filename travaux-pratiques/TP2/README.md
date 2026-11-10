# TP2 - Paddle model and PID position control

- `main.m` and `HapticPaddle.slx`: MATLAB/Simulink model of the paddle (inertia, viscous and dry friction, gravity) used to design and tune a PID controller.
- `haptic_controller.c`: firmware PID position controller (encoder or Hall feedback, filtered position, torque saturation, gains tunable from the PC).
- `tp2_4.m`: computes rise time, overshoot and settling time from a logged step response (expects a log CSV in a `log/` folder; logs are not included).
- `pid_block_diagram.png`: structure of the PID block used in the Simulink model.
