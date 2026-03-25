data = readtable(['log/log_2026-03-25_17-35-23.csv'], VariableNamingRule='preserve');

target = data.("pid_target_angle [deg]");
measured = data.("encoder_paddle_pos [deg]"); 
time = data.("timestamp [s]");

%% =========================
%  Compute performance metrics (DÉPLACÉ AVANT LE GRAPHE)
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
[max_value, max_idx] = max(measured); % J'ai ajouté 'max_idx' pour trouver l'instant du pic
t_max = time(max_idx); % Instant où l'overshoot est maximum
overshoot = (max_value - final_value) / step_amplitude * 100; % [%]

%% -------------------------
% Settling Time
% -------------------------
tolerance = 0.2;

% 1. Detect step start (0 -> 10 deg)
step_index = find(abs(target - target(1)) > 0.5, 1);
step_time  = time(step_index);

% 2. Final value (steady-state)
% (Déjà calculé plus haut)

% 3. Error relative to final value
error_signal = abs(measured - final_value);

% 4. Find settling time
settling_time = NaN;
t_settling_abs = NaN; % Variable ajoutée pour stocker le temps absolu pour le graphe

for i = step_index:length(time)
    % Condition: from this point onward, ALWAYS within band
    if all(error_signal(i:end) <= tolerance)
        settling_time = time(i) - step_time;
        t_settling_abs = time(i); % Temps sur l'axe des abscisses
        break;
    end
end

%% =========================
%  Display results in console
% =========================
fprintf('--- PID Performance ---\n');
fprintf('Rise time: %.3f s\n', rise_time);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Settling time: %.3f s\n', settling_time);

%% =========================
%  Plot step response
% =========================
figure;
plot(time, target, '--', 'LineWidth', 1.5); hold on;
plot(time, measured, 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('Angle [deg]');
title('PID Step Response (10° Step)');

% =========================
%  AJOUT DES TRAITS DEMANDÉS
% =========================

% 1. Trait pointillé vertical pour le Rise Time (t90)
% (J'ajoute aussi t10 en plus clair pour bien encadrer le "rise time")
xline(t10, ':', 'Color', [0.5 0.5 0.5]); 
xline(t90, '--g', 't_{90} (Rise Time)', 'LabelOrientation', 'horizontal', 'LineWidth', 1.5);

% 2. Trait pointillé vertical pour le Settling Time
if ~isnan(t_settling_abs)
    xline(t_settling_abs, '--m', 'Settling Time', 'LabelOrientation', 'horizontal', 'LineWidth', 1.5);
end

% 3. Trait vertical pour le pourcentage d'overshoot
% Trace une ligne rouge du "steady state" jusqu'au sommet du pic
plot([t_max, t_max], [final_value, max_value], '-r', 'LineWidth', 2);
% Ajoute le texte du pourcentage juste à côté de ce trait
text(t_max + 0.02, max_value - (max_value - final_value)/2, ...
    sprintf('%.1f%%', overshoot), 'Color', 'r', 'FontWeight', 'bold');



legend('Target', 'Measured', 'overshoot','Settling Time', 'Rise Time',  'Location', 'best');
hold off;