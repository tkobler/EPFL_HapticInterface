# TP3 - Virtual walls, compensation and K-B plots

`haptic_controller.c` builds on the TP2 PID code and adds:
- virtual viscoelastic walls at +/-15 degrees (spring K and damper B, with a filter on position and velocity);
- dry friction, viscous friction and gravity compensation, plus a stiction estimation routine;
- a K-B sweep: K is ramped up for a fixed B until the user flags vibration, then B is increased, giving 10 (B, K) stability points;
- experiment modes comparing filtering and sampling periods (350 us vs 10 ms), with or without compensation.
