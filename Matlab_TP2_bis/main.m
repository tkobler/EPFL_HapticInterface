%% Clean the workspace.
clc 
clear variables
close all

%% Set the simulation parameters.

% Motor parameters (faulhaber 2642W024CR), obtained from its datasheet.
%Lm      = 0.00055; % Phase-to-phase inductance [H]. Since the board is directly performing current/torque control, this can be ignored.
%Rm      = 5.78; % Phase-to-phase resistance [ohm]. Since the board is directly performing current/torque control, this can be ignored.
% k_tau	= 0.0346; % Torque constant [Nm/A]. Since the board is directly performing current/torque control, this can be ignored.
B_m     = 7.90E-07; % Viscous friction of the ball bearings [Nm/(rad/s)].
J_rotor = 1.1e-6; % Rotor inertia [kg.m^2].

% Paddle mechanical parts parameters, obtained from the CAD model.
r_p1	= 0.075; % Radius of the bottom arc of the paddle [m].
r_m     = 0.005; % Motor worm screw radius [m].
J_p     = 2.05404e-4; % Paddle inertia [kg*m^2]. 
J_m     = 0.144E-06; % Worm screw inertia [kg*m^2]. 1.44
B_p     = 0; % Paddle ball bearing viscous friction [Nm*s/rad]. Will be ignored for this simulation.
m_p     = 0.048; % Paddle mass [kg].
l       = 0.0251; % Distance between the center of rotation and the center of mass of the paddle [m].
r_p2    = 0.081; % Length of the paddle lever (to apply a force with the finger) [m].

% Environment parameters.
g       = 9.81; % Gravity [N/kg].

% Computed constants.
R = r_p1 / r_m; % Reduction ratio of the cable transmission [].
%************************* TO BE COMLPETED ********************************
J_eq  = (J_p + R^2*(J_rotor + J_m)); % Equivalent total inertia, computed on the paddle side [kg*m^2].
B_eq  = B_p + R^2*B_m; % Equivalent viscous friction coefficient, computed on the paddle side [Nm*s/rad].
Tau_f_dry = 0.00156; % Identified dry friction paddle side

phi_target = 10.00; % angle cible [deg]  % -> change to RAD if we use it ====================================

%% Run the Simulink simulation.
sim('HapticPaddle_vR2018b.mdl');

%% Plot the simulation results.
figure;

subplot(3,1,1)
hold on
plot(phi, 'g') % position réelle
plot(phi_target * ones(size(phi)), 'r--') % target constante
hold off

ylabel('\phi [rad]')
title('Angle')
legend('phi', 'phi Taarget')

subplot(3,1,2)
plot(omega)
ylabel('\omega [rad/s]')
title('Angular speed')

subplot(3,1,3)
plot(omega_dot)
ylabel('\omega_dot [rad/s^2]')
title('Angular acceleration')