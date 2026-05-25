



%{

%% ========================================================================
%% Chargement des données
%% ========================================================================
% --- Expérience S1_angle ---
data1 = readtable('logs/S1_angle.csv', 'VariableNamingRule', 'preserve');
time1 = data1.("timestamp [s]");
measured1 = data1.("encoder_paddle_pos [deg]");

% --- Expérience S1_comp ---
data2 = readtable(['logs/S1_comp.csv'], 'VariableNamingRule', 'preserve');
time2 = data2.("timestamp [s]");
measured2 = data2.("encoder_paddle_pos [deg]");

%% ========================================================================
%% Génération du Graphique (Plot)
%% ========================================================================
figure('Color', [1 1 1]); % Fond blanc pour un rendu plus propre

% 1. Tracé de la consigne (Target) - Basé sur S1 ou S2 selon ton choix
% Si la consigne est dans le fichier, pense à la réactiver (ex: data1.("pid_target_angle [deg]"))
% h_target = plot(time1, target, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5); hold on;
hold on; % Active la superposition des graphiques

% --- AJOUT DE LA LIGNE À 0 ---
% '--k' dessine une ligne en pointillés (--) et en noir (k)
% 'HandleVisibility', 'off' permet de ne pas polluer la légende avec cette ligne
yline(0, '--k', 'LineWidth', 1, 'HandleVisibility', 'off');

% 2. Tracé de l'expérience S1 (en bleu)
h_meas1 = plot(time1, measured1, 'b', 'LineWidth', 2);

% 3. Tracé de l'expérience S2 (en rouge/orange pour contraster)
h_meas2 = plot(time2, measured2, 'r', 'LineWidth', 2);

% Configuration des axes
grid on;
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Subject 1, Comparaison Between Angle Control And Enhanced Angle Control');

%% ========================================================================
%% Gestion de la Légende
%% ========================================================================
% On associe proprement chaque courbe à son étiquette dans la légende
legend([h_meas1, h_meas2], {'Position Control', 'Enhanced Position Control'}, 'Location', 'best');

% Si tu réactives la ligne 'Target', utilise plutôt cette légende :
% legend([h_target, h_meas1, h_meas2], {'Target', 'Measured S1', 'Measured S2'}, 'Location', 'best');

hold off;
%}


%% ========================================================================
%% Parameters
%% ========================================================================
DELTA_ANGLE     = 10;    % [deg]  minimum angle increase to detect onset
DELTA_TIME      = 5;     % [s]    time window within which increase must occur
WINDOW_DURATION = 5;     % [s]    duration of extracted window after onset

%% ========================================================================
%% Chargement des données
%% ========================================================================
% --- Expérience S1_angle (basic / position controller) ---
data1     = readtable('logs/S1_angle.csv', 'VariableNamingRule', 'preserve');
time1     = data1.("timestamp [s]");
measured1 = data1.("encoder_paddle_pos [deg]");

% --- Expérience S1_comp (enhanced controller) ---
data2     = readtable('logs/S1_comp.csv', 'VariableNamingRule', 'preserve');
time2     = data2.("timestamp [s]");
measured2 = data2.("encoder_paddle_pos [deg]");

% ── Add more subjects here and append to posCtrlData / enhCtrlData below ──
 data3 = readtable('logs/S2_angle.csv', 'VariableNamingRule', 'preserve');  
 time3 = data3.("timestamp [s]");
 measured3 = data3.("encoder_paddle_pos [deg]");

 data4 = readtable('logs/S2_comp.csv', 'VariableNamingRule', 'preserve');   
 time4 = data4.("timestamp [s]");
 measured4 = data4.("encoder_paddle_pos [deg]");

%% ========================================================================
%% Repetition Detection Function
%% ========================================================================
% For every sample i, look ahead within DELTA_TIME seconds.
% The FIRST sample j where angle(j) - angle(i) >= DELTA_ANGLE is the onset.
% After an onset at j, skip forward WINDOW_DURATION seconds to avoid
% re-detecting the same stroke.

    function [onsetIdx, windows] = detectRepetitions(t, angle, ...
            deltaAngle, deltaTime, windowDur)
        onsetIdx = [];
        n        = numel(t);
        i        = 1;
        while i <= n
            % Indices that fall within the look-ahead window
            lookAheadIdx = find((t >= t(i)) & (t <= t(i) + deltaTime));
            % Find the FIRST j in that window where the angle rise is enough
            jLocal = find(angle(lookAheadIdx) - angle(i) >= deltaAngle, 1, 'first');
            if ~isempty(jLocal)
                j = lookAheadIdx(jLocal);   % onset is at j, not i
                onsetIdx(end+1) = j;        %#ok<AGROW>
                % Skip past the extraction window
                skipTime = t(j) + 7;
                i = find(t >= skipTime, 1, 'first');
                if isempty(i), break; end
            else
                i = i + 1;
            end
        end

        % Build [tStart, tEnd] pairs for each detected window
        windows = zeros(numel(onsetIdx), 2);
        for k = 1:numel(onsetIdx)
            tStart       = t(onsetIdx(k));
            windows(k,:) = [tStart, tStart + windowDur];
        end
    end

%% ========================================================================
%% Run Detection on Every Subject / Controller
%% ========================================================================
[onsetIdx1, windows1] = detectRepetitions(time1, measured1, ...
    DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);
[onsetIdx2, windows2] = detectRepetitions(time2, measured2, ...
    DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);
[onsetIdx3, windows3] = detectRepetitions(time3, measured3, ...
    DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);
[onsetIdx4, windows4] = detectRepetitions(time4, measured4, ...
    DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);



% ── Detect for additional subjects and append their windows below ──
% [~, windows3] = detectRepetitions(time3, measured3, DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);
% [~, windows4] = detectRepetitions(time4, measured4, DELTA_ANGLE, DELTA_TIME, WINDOW_DURATION);

fprintf('Position Control (S1):          %d repetitions found\n', numel(onsetIdx1));
fprintf('Enhanced Position Control (S1): %d repetitions found\n', numel(onsetIdx2));

fprintf('Position Control (S2):          %d repetitions found\n', numel(onsetIdx3));
fprintf('Enhanced Position Control (S2): %d repetitions found\n', numel(onsetIdx4));


%% ========================================================================
%% Helper: draw coloured background patches
%% ========================================================================
    function drawRepetitionPatches(ax, windows, yLimits, colour)
        for k = 1:size(windows, 1)
            x = [windows(k,1), windows(k,2), windows(k,2), windows(k,1)];
            y = [yLimits(1),   yLimits(1),   yLimits(2),   yLimits(2)  ];
            patch(ax, x, y, colour, ...
                'EdgeColor',       'none', ...
                'FaceAlpha',       0.18, ...
                'HandleVisibility','off');
        end
    end

%% ========================================================================
%% Figure 1 – Time-series with coloured repetition backgrounds
%% ========================================================================
fig1 = figure('Color', [1 1 1], 'Name', 'Time Series + Repetitions');
set(fig1, 'Position', [100 100 1200 400]);
ax1  = axes('Parent', fig1);
hold(ax1, 'on');

allAngle = [measured1; measured2];
yPad     = 0.05 * (max(allAngle) - min(allAngle));
yLim     = [min(allAngle) - yPad,  max(allAngle) + yPad];

% Coloured backgrounds – drawn FIRST so signal lines sit on top
drawRepetitionPatches(ax1, windows1, yLim, [0.20, 0.45, 0.87]);  % blue  = pos ctrl
drawRepetitionPatches(ax1, windows2, yLim, [0.87, 0.25, 0.20]);  % red   = enh ctrl

yline(ax1, 0, '--k', 'LineWidth', 1, 'HandleVisibility', 'off');

h1 = plot(ax1, time1, measured1, 'b', 'LineWidth', 2);
h2 = plot(ax1, time2, measured2, 'r', 'LineWidth', 2);

% Dummy patches for legend
hPatch1 = patch(ax1, NaN, NaN, [0.20, 0.45, 0.87], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
hPatch2 = patch(ax1, NaN, NaN, [0.87, 0.25, 0.20], 'FaceAlpha', 0.35, 'EdgeColor', 'none');

ylim(ax1, yLim);
grid(ax1, 'on');
xlabel(ax1, 'Time [s]');
ylabel(ax1, 'Angle [deg]');
title(ax1, 'Subject 1 – Position vs Enhanced Position Control');
legend(ax1, [h1, h2, hPatch1, hPatch2], ...
    {'Position Control', 'Enhanced Position Control', ...
     'Repetitions (Pos.)', 'Repetitions (Enh.)'}, ...
    'Location', 'best');
hold(ax1, 'off');
%%%
fig2 = figure('Color', [1 1 1], 'Name', 'Time Series + Repetitions');
set(fig2, 'Position', [100 100 1200 400]);
ax1  = axes('Parent', fig2);
hold(ax1, 'on');

allAngle = [measured3; measured4];
yPad     = 0.05 * (max(allAngle) - min(allAngle));
yLim     = [min(allAngle) - yPad,  max(allAngle) + yPad];

% Coloured backgrounds – drawn FIRST so signal lines sit on top
drawRepetitionPatches(ax1, windows3, yLim, [0.20, 0.45, 0.87]);  % blue  = pos ctrl
drawRepetitionPatches(ax1, windows4, yLim, [0.87, 0.25, 0.20]);  % red   = enh ctrl

yline(ax1, 0, '--k', 'LineWidth', 1, 'HandleVisibility', 'off');

h1 = plot(ax1, time3, measured3, 'b', 'LineWidth', 2);
h2 = plot(ax1, time4, measured4, 'r', 'LineWidth', 2);

% Dummy patches for legend
hPatch1 = patch(ax1, NaN, NaN, [0.20, 0.45, 0.87], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
hPatch2 = patch(ax1, NaN, NaN, [0.87, 0.25, 0.20], 'FaceAlpha', 0.35, 'EdgeColor', 'none');

ylim(ax1, yLim);
grid(ax1, 'on');
xlabel(ax1, 'Time [s]');
ylabel(ax1, 'Angle [deg]');
title(ax1, 'Subject 2 – Position vs Enhanced Position Control');
legend(ax1, [h1, h2, hPatch1, hPatch2], ...
    {'Position Control', 'Enhanced Position Control', ...
     'Repetitions (Pos.)', 'Repetitions (Enh.)'}, ...
    'Location', 'best');
hold(ax1, 'off');

%% ========================================================================
%% Extract angle samples that fall inside detected windows
%% ========================================================================
    function pooled = extractAndPool(timeCell, angleCell, windowsCell)
        % timeCell   : cell array of time vectors,  one per subject
        % angleCell  : cell array of angle vectors, one per subject
        % windowsCell: cell array of Nx2 window matrices, one per subject
        % Returns a single column vector of ALL angle samples from ALL windows
        % across ALL subjects for this controller.
        pooled = [];
        for s = 1:numel(timeCell)
            t   = timeCell{s};
            ang = angleCell{s};
            W   = windowsCell{s};
            for k = 1:size(W, 1)
                mask   = (t >= W(k,1)) & (t <= W(k,2));
                pooled = [pooled; ang(mask)]; %#ok<AGROW>
            end
        end
    end

% ── Bundle all subjects per controller ──
% Position control  (blue background)
posCtrl_times   = {time1, time3};       % add time3, time5 … for more subjects
posCtrl_angles  = {measured1, measured3};
posCtrl_windows = {windows1, windows3};

% Enhanced control  (red background)
enhCtrl_times   = {time2, time4};       % add time4, time6 … for more subjects
enhCtrl_angles  = {measured2, measured4};
enhCtrl_windows = {windows2, windows4};

pooledPos = extractAndPool(posCtrl_times,  posCtrl_angles,  posCtrl_windows);
pooledEnh = extractAndPool(enhCtrl_times,  enhCtrl_angles,  enhCtrl_windows);

%% ========================================================================
%% Figure 2 – Box Plot: both controllers side-by-side (all subjects pooled)
%% ========================================================================
if isempty(pooledPos) && isempty(pooledEnh)
    warning('No repetition data found – skipping box plot.');
else
    % Assemble combined vector + group labels
    nPos = numel(pooledPos);
    nEnh = numel(pooledEnh);
    combinedVals  = [pooledPos;    pooledEnh   ];
    combinedGroup = [ones(nPos,1); 2*ones(nEnh,1)];

    fig3 = figure('Color', [1 1 1], 'Name', 'Box Plot – All Subjects');
    ax2  = axes('Parent', fig3);
    hold(ax2, 'on');

    bp = boxplot(ax2, combinedVals, combinedGroup, ...
        'Colors',    [0.20, 0.45, 0.87; 0.87, 0.25, 0.20], ...
        'Symbol',    '+', ...
        'Widths',    0.4, ...
        'OutlierSize', 4, ...
        'Labels',    {'Position Control', 'Enhanced Position Control'});
    set(bp, 'LineWidth', 1.8);

    % Colour the box faces to match the time-series patches
    hBoxes = findobj(ax2, 'Tag', 'Box');
    % boxplot orders boxes right-to-left, so hBoxes(1)=Enhanced, hBoxes(2)=Position
    if numel(hBoxes) >= 2
        patch(ax2, get(hBoxes(2), 'XData'), get(hBoxes(2), 'YData'), ...
            [0.20, 0.45, 0.87], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
        patch(ax2, get(hBoxes(1), 'XData'), get(hBoxes(1), 'YData'), ...
            [0.87, 0.25, 0.20], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    end

    grid(ax2, 'on');
    ylabel(ax2, 'Angle [deg]');
    title(ax2, 'Angle Distribution – All Subjects, Both Controllers');
    hold(ax2, 'off');
end
