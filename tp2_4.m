

data = readtable('log/log_2026-03-24_11-16-26.csv', VariableNamingRule='preserve');


target = data.("pid_target_angle [deg]");
measured = data.("encoder_paddle_pos [deg]"); 
time = data.("timestamp [s]");


%% =========================
%  Plot step response
% =========================
figure;
plot(time, target, '--', 'LineWidth', 1.5); hold on;
plot(time, measured, 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Target', 'Measured');
title('PID Step Response (10° Step)');

%% =========================
%  Compute performance metrics
% =========================

% Final value (steady-state approximation)
final_value = mean(measured(end-50:end));

% Step amplitude (should be ~10°)
step_amplitude = final_value - measured(1);

% -------------------------
% Rise Time (10% → 90%)
% -------------------------
y10 = measured(1) + 0.1 * step_amplitude;
y90 = measured(1) + 0.9 * step_amplitude;

t10 = time(find(measured >= y10, 1));
t90 = time(find(measured >= y90, 1));

rise_time = t90 - t10;

% -------------------------
% Overshoot
% -------------------------
max_value = max(measured);
overshoot = (max_value - final_value) / step_amplitude * 100; % [%]


%% -------------------------
% Settling Time
% -------------------------

tolerance = 0.2;

% 1. Detect step start (0 -> 10 deg)
step_index = find(abs(target - target(1)) > 0.5, 1);
step_time  = time(step_index);

% 2. Final value (steady-state)
final_value = mean(measured(end-50:end));

% 3. Error relative to final value
error_signal = abs(measured - final_value);

% 4. Find settling time
settling_time = NaN;

for i = step_index:length(time)
    % Condition: from this point onward, ALWAYS within band
    if all(error_signal(i:end) <= tolerance)
        settling_time = time(i) - step_time;
        break;
    end
end

fprintf('Settling time: %.3f s\n', settling_time);

%% =========================
%  Display results
% =========================
fprintf('--- PID Performance ---\n');
fprintf('Rise time: %.3f s\n', rise_time);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Settling time: %.3f s\n', settling_time);

%% =========================
%  Plot error band (optional)
% =========================
figure;
plot(time, measured - final_value, 'LineWidth', 2); hold on;
yline(0.2, '--r');
yline(-0.2, '--r');

grid on;
xlabel('Time [s]');
ylabel('Error [deg]');
title('Settling Band (±0.2°)');
