%% ArUco low-light study: batch analysis of all trials
% -------------------------------------------------------------------------
% Loads every trial bag in ROOTS, computes the detection rate and the pose
% error against ground truth, and compares
%   - baseline vs dark lighting
%   - sun angle (with supplementary light none / low / med / high)
%   - how much supplementary light recovers detection, and where
%   - easier vs harder lighting scenarios
%   - baseline vs dark at chosen sun angles (ANGLE_FIGURES)
%
% Ground truth
%   baseline set : each bag's own OptiTrack data (camera + marker rigid bodies)
%   dark set     : the OptiTrack data in GT_BAG (baseline_dark_set_ground_truth):
%                  camera and marker rigid-body poses from that bag (known marker
%                  offset applied) are the ground truth for EVERY dark trial.
%                  A dark bag's own OptiTrack data is never used as ground truth.
%
% Position error is in the camera optical frame (X lateral, Y vertical,
% Z depth), in mm. Orientation error is about the ground-truth marker axes,
% in degrees.
%
% Figures (one window each, all graphs for a section tiled inside it):
%   1  Detection rate, every baseline and dark trial
%   2  Sun angle effect, rows = supplementary light level
%   3  Supplementary light recovery heat maps
%   4  Easier vs harder scenarios
%   5+ Baseline vs dark at each angle in ANGLE_FIGURES
%   then one figure per sun angle: ArUco vs ground-truth X/Y/Z position over time,
%        and one per sun angle for orientation about marker X/Y/Z,
%        baseline and dark overlaid, columns = supplementary light level
%   then the OBJECTIVE ANALYSES (printed tables, CSVs and figures):
%     Obj 4  repeatability    noise floor from the repeated reference trials, used
%                             to decide whether any change is larger than run-to-run
%                             variation (no significance tests yet)
%     Obj 1  degradation      every no-supplementary trial vs the ideal reference
%                             (Ambient, no sun, no supplementary light), classified
%                             into failure modes (no detection / intermittent /
%                             less accurate / less stable / wrong-ID detections)
%     Obj 2  lamp design      supplementary light alone (no sun) vs no light, OptiTrack
%                             dropouts with the lamp on vs off, requirement checklist
%     Obj 3  mitigation       change vs no supplementary light at the same sun angle
%                             (detection, accuracy, stability, orientation jumps),
%                             accuracy-change heat maps, detection vs illuminance
%     Obj 5  suitability      pass / fail per sun angle against availability,
%                             accuracy, stability, real-time and OptiTrack criteria
%
% Per-trial additions: detection counts only the expected marker ID (MARKER_ID),
% wrong-ID detections are counted separately; 3D position error; orientation-jump
% (solution flip) rate; OptiTrack dropout percentage; processing time if the bag
% has a processing-time topic (see time_aruco_pipeline.py otherwise).
%
% Requires: MATLAB R2021a+ with ROS Toolbox (ros2bagreader).
% -------------------------------------------------------------------------

clear; clc; close all;

%% ============================ SETTINGS ===================================
% Folders that contain the bag folders (baseline and dark sets can be in
% different folders; every bag is looked up in all of them).
ROOTS  = {fullfile(pwd, 'baseline'), fullfile(pwd, 'dark')};
GT_BAG = 'baseline_dark_set_ground_truth'; % reference ground truth for the dark set

% Ground-truth source per set:
%   'own'       = the bag's own OptiTrack data, interpolated to every ArUco frame
%   'static'    = the bag's own OptiTrack data reduced to one median pose (valid only
%                 because the marker is static; removes OptiTrack noise, so baseline
%                 errors can be compared fairly with the dark set's fixed reference)
%   'reference' = the OptiTrack data in GT_BAG (fixed camera + marker poses)
% The dark set always uses 'reference' (enforced below).
GT_MODE = struct('baseline', 'static', 'dark', 'reference');

% Trial list (bag folder | set | sun angle | supplementary light).
% Sun angle: degrees, or 'none' = no sun (the "base" lighting condition).
% Supplementary light: 'none', 'low', 'med' or 'high'.
% Repeats of the same condition (base, base1, base2) are averaged.
% Set TRIALS = {} to build the list from the folder names instead.
TRIALS = {
    % ---------------- baseline set (own OptiTrack ground truth) ----------------
    'baseline_improved',            'baseline', 'none', 'none';   % replaces baseline_base
    'baseline_base1',               'baseline', 'none', 'none';
    'baseline_base2',               'baseline', 'none', 'none';
    'baseline_low',                 'baseline', 'none', 'low';
    'baseline_med',                 'baseline', 'none', 'med';
    'baseline_high',                'baseline', 'none', 'high';
    'baseline_sun',                 'baseline', 0,      'none';   % ASSUMED sun 0 deg, no sup
    'baseline_sun_0_deg_sup_low',   'baseline', 0,      'low';
    'baseline_sun_0_deg_sup_med',   'baseline', 0,      'med';
    'baseline_sun_0_deg_sup_high',  'baseline', 0,      'high';
    'baseline_sun_15_deg',          'baseline', 15,     'none';
    'baseline_sun_15_deg_sup_low',  'baseline', 15,     'low';    
    'baseline_sun_15_deg_sup_med',  'baseline', 15,     'med';
    'baseline_sun_15_deg_sup_high', 'baseline', 15,     'high';
    'baseline_sun_30_deg',          'baseline', 30,     'none';
    'baseline_sun_30_deg_sup_low',  'baseline', 30,     'low';
    'baseline_sun_30_deg_sup_med',  'baseline', 30,     'med';
    'baseline_sun_30_deg_sup_high', 'baseline', 30,     'high';
    'baseline_sun_45',              'baseline', 45,     'none';
    'baseline_sun_45_deg_sup_low',  'baseline', 45,     'low';
    'baseline_sun_45_deg_sup_med',  'baseline', 45,     'med';
    'baseline_sun_45_deg_sup_high', 'baseline', 45,     'high';
    'baseline_sun_60_deg',          'baseline', 60,     'none';
    'baseline_sun_60_deg_sup_low',  'baseline', 60,     'low';
    'baseline_sun_60_deg_sup_med',  'baseline', 60,     'med';
    'baseline_sun_60_deg_sup_high', 'baseline', 60,     'high';
    'baseline_sun_75_deg',          'baseline', 75,     'none';
    'baseline_sun_75_deg_sup_low',  'baseline', 75,     'low';
    'baseline_sun_75_deg_sup_med',  'baseline', 75,     'med';
    'baseline_sun_75_deg_sup_high', 'baseline', 75,     'high';
    % 'baseline_base',              'baseline', 'none', 'none';   % replaced by baseline_improved
    % ---------------- dark set (reference ground truth) ------------------------
    'dark_base',                    'dark',     'none', 'none';
    'dark_low',                     'dark',     'none', 'low';
    'dark_med',                     'dark',     'none', 'med';
    'dark_high',                    'dark',     'none', 'high';
    'dark_sun_0_deg',               'dark',     0,      'none';
    'dark_sun_0_deg_sup_low',       'dark',     0,      'low';
    'dark_sun_0_deg_sup_med',       'dark',     0,      'med';
    'dark_sun_0_deg_sup_high',      'dark',     0,      'high';
    'dark_sun_15_deg',              'dark',     15,     'none';
    'dark_sun_15_deg_sup_low',      'dark',     15,     'low';
    'dark_sun_15_deg_sup_med',      'dark',     15,     'med';
    'dark_sun_15_deg_sup_high',     'dark',     15,     'high';
    'dark_sun_30_deg',              'dark',     30,     'none';
    'dark_sun_30_deg_sup_low',      'dark',     30,     'low';
    'dark_sun_30_deg_sup_med',      'dark',     30,     'med';
    'dark_sun_30_deg_sup_high',     'dark',     30,     'high';
    'dark_sun_45_deg',              'dark',     45,     'none';
    'dark_sun_45_deg_sup_low',      'dark',     45,     'low';
    'dark_sun_45_deg_sup_med',      'dark',     45,     'med';
    'dark_sun_45_deg_sup_high',     'dark',     45,     'high';
    'dark_sun_60_deg_',             'dark',     60,     'none';
    'dark_sun_60_deg_sup_low',      'dark',     60,     'low';
    'dark_sun_60_deg_sup_med',      'dark',     60,     'med';
    'dark_sun_60_deg_sup_high',     'dark',     60,     'high';
    'dark_sun_75_deg',              'dark',     75,     'none';
    'dark_sun_75_deg_sup_low',      'dark',     75,     'low';
    'dark_sun_75_deg_sup_med',      'dark',     75,     'med';
    'dark_sun_75_deg_sup_high',     'dark',     75,     'high';
};

SUP_LEVELS  = {'none', 'low', 'med', 'high'};  % order used in every plot
ANGLE_ORDER = {};          % e.g. {'none','0','15','30'}; empty = no sun first, then ascending
TS_ANGLES     = {};          % sun angles for the position-over-time figures; {} = all
ANGLE_FIGURES = {'none', '0'};   % one baseline-vs-dark figure per sun angle:
                                 % 'none' = base lighting (no sun), '0' = sun at 0 deg

HARD_THRESHOLD  = 90;      % % detection with no supplementary light below which
                           % a scenario counts as "hard"
REMOVE_OUTLIERS = false;   % true = drop single-frame ArUco failures from the error stats
SAVE_RESULTS    = true;    % PNGs + CSVs in <first ROOTS folder>/analysis_results

% ---------------- settings for the objective analyses ----------------
MARKER_ID = 0;             % ID of the printed marker; any other ID = misidentification

% Measured illuminance at the marker centre (lux) with the supplementary light only
% (no sun), in SUP_LEVELS order {none, low, med, high}. NaN = not measured.
LUX_SUP = struct('baseline', [836 1224 1380 1690], ...
                 'dark',     [46  1000  1300  1765]);   % <-- add dark-box values if measured
SUN_LUX = 12000;           % measured simulated-sun illuminance at the marker (lux)

% Suitability criteria (objective 5). NaN = not set yet: reported as "target not set".
ACCURACY_TARGET_MM  = NaN; % max 3D position RMSE for a usable ground truth (mm)
ACCURACY_TARGET_DEG = NaN; % max total orientation RMSE (deg)
FRAME_RATE_HZ       = 30;  % camera frame rate: processing must finish within one frame
INTERFERENCE_TOL_PCT = 1.0;% max extra OptiTrack dropout (percentage points) with lamp on

% Noise floor (objective 4): a change counts as real only if it is larger than
% NOISE_K x the spread of the repeated reference trials, and never smaller than
% these minimum values (used when there are fewer than two repeats).
NOISE_K   = 2;
MIN_NOISE = struct('pos_mm', 0.5, 'ori_deg', 0.2, 'flip_pct', 1.0, 'det_pct', 2.0);

% Camera rigid body -> camera optical frame
BC_T_C = [ 0.99931745, -0.01430626,  0.03405815,  0.00422558;
           0.03325054, -0.05333160, -0.99802312, -0.01757960;
           0.01609435,  0.99847437, -0.05281951,  0.01983206;
           0 0 0 1 ];

% Marker rigid body -> ArUco centre
BM_T_M = eye(4);
BM_T_M(1:3,1:3) = [0 1 0; 0 0 1; 1 0 0];
BM_T_M(1:3,4)   = [0.075; -0.0146; -0.117];

% Processing parameters
P.timeOffset       = 0.0;   % s added to ArUco stamps to match the mocap clock
P.minFrozenSamples = 5;     % identical consecutive mocap poses = lost tracking
P.jumpThreshold    = 0.02;  % m between consecutive mocap samples
P.dropoutMargin    = 0.10;  % s excluded either side of a mocap dropout
P.outlierWindow    = 7;     % frames in running median for outlier detection
P.kMAD             = 6;     % robust outlier threshold multiplier
P.minOriOutlier    = 2.0;   % deg
P.minPosOutlier    = 10.0;  % mm
P.removeOutliers   = REMOVE_OUTLIERS;
P.topicAruco       = '/aruco/pose';
P.topicDetected    = '/aruco/detected';
P.topicTotal       = '/aruco/total_frames';
P.topicDetFrames   = '/aruco/detected_frames';
P.topicId          = '/aruco/id';
P.topicProc        = '/aruco/processing_time_ms';  % optional; absent in older bags
P.markerId         = MARKER_ID;
P.topicCamRB       = '/vrpn_mocap/camera_rigidbody/pose';
P.topicMarkerRB    = '/vrpn_mocap/marker_rigidbody/pose';

SETS   = {'baseline', 'dark'};
SETCOL = [0.00 0.45 0.74; 0.25 0.25 0.25];
SETMK  = {'o-', 's--'};
SUPCOL = [0.65 0.65 0.65; 0.93 0.69 0.13; 0.85 0.33 0.10; 0.64 0.08 0.18];

%% ============================ TRIAL LIST =================================
if isempty(TRIALS)
    fprintf('No TRIALS table given: parsing bag folder names.\n');
    TRIALS = cell(0, 4);
    for r = 1:numel(ROOTS), TRIALS = [TRIALS; autoTrials(ROOTS{r}, GT_BAG)]; end %#ok<AGROW>
end
TRIALS = normaliseTrials(TRIALS, GT_BAG);
nT = size(TRIALS, 1);
if nT == 0
    error('No trial bags found. Set ROOTS or fill in the TRIALS table.');
end

fprintf('\n%d trials:\n', nT);
fprintf('  %-40s %-9s %-8s %s\n', 'bag', 'set', 'sun', 'sup');
for i = 1:nT
    fprintf('  %-40s %-9s %-8s %s\n', TRIALS{i,1}, TRIALS{i,2}, ...
            angLabel(TRIALS{i,3}), TRIALS{i,4});
end
fprintf('  Check this list. If a condition is wrong, fill in the TRIALS table.\n');

angOrder  = angleOrder(TRIALS(:,3), ANGLE_ORDER);
angLabels = cellfun(@angLabel, angOrder, 'UniformOutput', false);

%% ======================= REFERENCE GROUND TRUTH ==========================
GT_MODE.dark = 'reference';    % every dark trial is compared against GT_BAG's OptiTrack
ref = [];
refSets = SETS(cellfun(@(s) strcmp(GT_MODE.(s), 'reference'), SETS));
if any(ismember(TRIALS(:,2), refSets))
    ref = loadReference(bagPath(ROOTS, GT_BAG), BC_T_C, BM_T_M, P);
    fprintf(['\nReference ground truth from %s (%s, %d samples, known marker offset applied):\n' ...
             '  marker at [%.1f %.1f %.1f] mm in the camera frame, ' ...
             'OptiTrack spread %.2f mm\n'], GT_BAG, ref.source, ref.n, ...
            1000 * ref.C_T_M(1:3,4), ref.spread_mm);
    fprintf('  Used as the ground truth for all %d dark trials.\n', ...
            sum(strcmp(TRIALS(:,2), 'dark')));
end

%% ============================ ANALYSE ====================================
fprintf('\nAnalysing trials...\n');
results = cell(nT, 1);
TS = cell(nT, 1);          % per-trial position time series
for i = 1:nT
    fprintf('  [%2d/%2d] %-40s', i, nT, TRIALS{i,1});
    mode = GT_MODE.(TRIALS{i,2});
    try
        [R, TS{i}] = analyseTrial(bagPath(ROOTS, TRIALS{i,1}), mode, ref, BC_T_C, BM_T_M, P);
    catch ME
        R = emptyResult();
        TS{i} = [];
        R.note = ['failed: ' ME.message];
    end
    R.bag = TRIALS{i,1}; R.set = TRIALS{i,2};
    R.sun = TRIALS{i,3}; R.sup = TRIALS{i,4};
    results{i} = R;
    if strcmp(mode, 'reference'), gtTag = ['GT: ' GT_BAG]; else, gtTag = 'GT: own OptiTrack'; end
    fprintf(' det %5.1f %% | RMSE X %5.1f Y %5.1f Z %5.1f mm | ori %5.2f deg | %s  %s\n', ...
            R.detRate, R.posX_rmse, R.posY_rmse, R.posZ_rmse, R.oriTot_rmse, gtTag, R.note);
end
S = resultsToTable(results);

%% ============================ PRINTED TABLES =============================
fprintf('\n===================== DETECTION RATE: BASELINE =====================\n');
printDetection(S, 'baseline', angOrder, SUP_LEVELS);
fprintf('\n======================= DETECTION RATE: DARK ========================\n');
printDetection(S, 'dark', angOrder, SUP_LEVELS);

for s = 1:numel(SETS)
    idx = sortedIdx(S, SETS{s}, angOrder, SUP_LEVELS);
    printStats(S, idx, sprintf('POSE ERROR, %s trials (mean / std / RMSE)', upper(SETS{s})));
end

camMoved = find(S.camMove_mm > 5 | S.camMove_deg > 0.5);
for i = camMoved(:)'
    fprintf(['WARNING: %s: camera rigid body is %.1f mm / %.2f deg away from the ' ...
             'reference bag. The reference ground truth may not apply.\n'], ...
            S.bag{i}, S.camMove_mm(i), S.camMove_deg(i));
end

supsP = SUP_LEVELS(ismember(SUP_LEVELS, S.sup));
nA = numel(angOrder);
figs = gobjects(0);

%% ================ FIGURE 1: DETECTION RATE, ALL TRIALS ===================
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
figs(end+1) = figure('Name', 'Detection rate', 'Color', 'w', 'Position', [40 40 1300 760]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for s = 1:numel(SETS)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    idx = sortedIdx(S, SETS{s}, angOrder, SUP_LEVELS);
    if isempty(idx)
        noData(ax, ['no ' SETS{s} ' trials']); continue;
    end
    y = S.detRate(idx);
    bar(ax, y, 'FaceColor', SETCOL(s,:));
    for k = 1:numel(idx)
        if ~isnan(y(k))
            text(ax, k, y(k) + 2, sprintf('%.0f', y(k)), ...
                 'HorizontalAlignment', 'center', 'FontSize', 8);
        end
    end
    labels = arrayfun(@(i) sprintf('%s | sup %s', angLabel(S.sun{i}), S.sup{i}), ...
                      idx, 'UniformOutput', false);
    xticks(ax, 1:numel(idx)); xticklabels(ax, labels); xtickangle(ax, 40);
    xlim(ax, [0.4 numel(idx) + 0.6]); ylim(ax, [0 110]);
    yline(ax, HARD_THRESHOLD, ':', sprintf('%g %%', HARD_THRESHOLD));
    ylabel(ax, 'Detection rate (%)');
    title(ax, sprintf('%s: detection rate (mean %.1f %%)', upper(SETS{s}), ...
                      mean(y, 'omitnan')));
end
title(tl, 'Detection rate of every trial');

%% =================== FIGURE 2: SUN ANGLE EFFECT ==========================
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
% Rows: supplementary light level.
% Columns: detection, X / Y / Z position RMSE (camera frame), orientation RMSE.
mets = {'detRate',     'Detection rate (%)';
        'posX_rmse',   'X lateral RMSE (mm)';
        'posY_rmse',   'Y vertical RMSE (mm)';
        'posZ_rmse',   'Z depth RMSE (mm)';
        'oriTot_rmse', 'Orientation RMSE (deg)'};
nM = size(mets, 1);

fprintf('\n=================== SUN ANGLE EFFECT (baseline B / dark D) ===================\n');
for r = 1:numel(supsP)
    fprintf('\n  Supplementary light: %s\n', supsP{r});
    fprintf('  %-14s | %7s %7s | %7s %7s | %7s %7s | %7s %7s | %7s %7s\n', 'sun', ...
            'det B', 'det D', 'X B', 'X D', 'Y B', 'Y D', 'Z B', 'Z D', 'ori B', 'ori D');
    for a = 1:nA
        v = zeros(1, 2 * nM);
        for c = 1:nM
            for s = 1:2
                v(2*(c-1) + s) = getMetric(S, SETS{s}, angOrder{a}, supsP{r}, mets{c,1});
            end
        end
        if all(isnan(v)), continue; end
        fprintf(['  %-14s | %6.1f%% %6.1f%% | %7.1f %7.1f | %7.1f %7.1f | %7.1f %7.1f' ...
                 ' | %7.2f %7.2f\n'], angLabels{a}, v);
    end
end
fprintf('  (X / Y / Z = position RMSE in mm, camera frame; ori = orientation RMSE in deg)\n');

if ~isempty(supsP)
    figs(end+1) = figure('Name', 'Sun angle effect', 'Color', 'w', ...
                         'Position', [30 40 1800 min(230 * numel(supsP) + 90, 1000)]);
    tl = tiledlayout(numel(supsP), nM, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:numel(supsP)
        for c = 1:nM
            ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
            for s = 1:2
                y = arrayfun(@(a) getMetric(S, SETS{s}, angOrder{a}, supsP{r}, mets{c,1}), 1:nA);
                plot(ax, 1:nA, y, SETMK{s}, 'Color', SETCOL(s,:), 'LineWidth', 1.5, ...
                     'MarkerFaceColor', SETCOL(s,:), 'DisplayName', SETS{s});
            end
            xticks(ax, 1:nA); xticklabels(ax, angLabels); xlim(ax, [0.5 nA + 0.5]);
            if nA > 4, xtickangle(ax, 30); end
            if c == 1
                ylim(ax, [0 105]);
                hline(ax, HARD_THRESHOLD);
                ylabel(ax, sprintf('sup light: %s', supsP{r}), 'FontWeight', 'bold');
            end
            if r == 1, title(ax, mets{c,2}); end
            if r == numel(supsP), xlabel(ax, 'Sun angle'); end
            if r == 1 && c == 1, legend(ax, 'Location', 'best'); end
        end
    end
    title(tl, ['Effect of sun angle at each supplementary light level (baseline vs dark), ' ...
               'position error per camera axis']);
end

%% ============ FIGURE 3: SUPPLEMENTARY LIGHT RECOVERY HEAT MAPS ===========
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
D = cell(1, 2);
for s = 1:2
    D{s} = nan(numel(supsP), nA);
    for r = 1:numel(supsP)
        for a = 1:nA
            D{s}(r, a) = getMetric(S, SETS{s}, angOrder{a}, supsP{r}, 'detRate');
        end
    end
end
iNone = find(strcmp(supsP, 'none'));

if ~isempty(supsP)
    figs(end+1) = figure('Name', 'Supplementary light recovery', 'Color', 'w', ...
                         'Position', [80 40 1300 800]);
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for s = 1:2
        ax = nexttile(tl, s);
        drawHeat(ax, D{s}, angLabels, supsP, [0 100], parula(256), '%.0f');
        title(ax, sprintf('%s: detection rate (%%)', upper(SETS{s})));
    end
    for s = 1:2
        ax = nexttile(tl, 2 + s);
        if isempty(iNone)
            noData(ax, 'no "none" supplementary light trials to compare against');
            continue;
        end
        Rv = D{s} - D{s}(iNone, :);
        m = max(abs(Rv(:)));
        if isempty(m) || isnan(m) || m == 0, m = 1; end
        drawHeat(ax, Rv, angLabels, supsP, [-m m], divMap(256), '%+.0f');
        title(ax, sprintf('%s: change vs no supplementary light (pp)', upper(SETS{s})));
    end
    title(tl, 'How much supplementary light recovers detection');
end

%% ================ FIGURE 4: EASIER vs HARDER SCENARIOS ===================
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
SC = scenarioTable(S, SETS, angOrder, supsP, HARD_THRESHOLD);

fprintf('\n============ LIGHTING SCENARIOS (hard = < %g %% detection with no sup light) ============\n', ...
        HARD_THRESHOLD);
fprintf('  %-22s %-8s', 'scenario', 'class');
hdr = strcat('det_', supsP);
fprintf(' %9s', hdr{:});
fprintf('  %-15s %s\n', 'recovered by', 'best');
for k = 1:numel(SC.label)
    fprintf('  %-22s %-8s', SC.label{k}, SC.cls{k});
    fprintf(' %8.1f%%', SC.det(k, :));
    fprintf('  %-15s %s\n', SC.rec{k}, SC.best{k});
end

figs(end+1) = figure('Name', 'All scenarios', 'Color', 'w', ...
                     'Position', [100 30 1700 1000]);
tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
rowsSC = {'det', 'Detection rate (%)';
          'px',  'X lateral RMSE (mm)';
          'py',  'Y vertical RMSE (mm)';
          'pz',  'Z depth RMSE (mm)'};
lbl = strrep(SC.label, 'baseline,', 'ambient,');      % match the report's wording
iDark = find(startsWith(SC.label, 'dark'), 1);        % first Dark scenario
for rr = 1:size(rowsSC, 1)
    ax = nexttile(tl);
    groupedBar(ax, lbl, SC.(rowsSC{rr,1}), supsP, SUPCOL);
    ylabel(ax, rowsSC{rr,2});
    if ~isempty(iDark) && iDark > 1                   % divider: Ambient | Dark
        xl = xline(ax, iDark - 0.5, 'k-', 'LineWidth', 1.2);
        xl.HandleVisibility = 'off';
    end
    if rr == 1
        ylim(ax, [0 110]);
        hline(ax, HARD_THRESHOLD);                    % 90 % reference line
        lg = legend(ax, strcat('sup:', {' '}, supsP));
        lg.Layout.Tile = 'east';                      % one legend, tiles stay aligned
    end
    if rr < size(rowsSC, 1), xticklabels(ax, {}); end % sun-angle labels on bottom row only
end
title(tl, ['Detection and X / Y / Z position RMSE for every lighting condition, ' ...
           'by supplementary light level']);

%% ======= FIGURES 5+: BASELINE vs DARK AT CHOSEN SUN ANGLES ===============
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
for k = 1:numel(ANGLE_FIGURES)
    sun = normAngle(ANGLE_FIGURES{k});
    f = compareFigure(S, sun, SUP_LEVELS, SETS, SETCOL, HARD_THRESHOLD);
    if ~isempty(f), figs(end+1) = f; end %#ok<SAGROW>
end

%% == FIGURES: ArUco vs GROUND TRUTH OVER TIME (POSITION, ORIENTATION), PER SUN ANGLE ==
% One figure per sun angle. Columns: supplementary light none / low / med / high.
% Rows: X lateral, Y vertical, Z depth. Baseline and dark are overlaid in each tile:
%   baseline: ArUco = blue dots,   ground truth = solid black line
%   dark:     ArUco = orange dots, ground truth = dashed green line
% Position figures: marker position in the camera frame (mm).
% Orientation figures: marker rotation about its X / Y / Z axes (deg), relative
% to the reference ground-truth orientation (so 0 = the reference pose).
% Mean error (ArUco - ground truth) for each set is printed in the set's colour;
% it is the same value as in the tables.
% If a condition has repeats, the first bag in TRIALS is shown.
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
tsAngles = angOrder;
if ~isempty(TS_ANGLES)
    tsAngles = cellfun(@normAngle, TS_ANGLES(:)', 'UniformOutput', false);
end
ax3    = 'XYZ';
% quantity | ArUco field | GT field | stats prefix | axis labels | unit | mean fmt | std fmt | title text
KINDS = {
  'Position',    'est',    'gt',    'pos', {'X lateral', 'Y vertical', 'Z depth'}, ...
                 'mm',  '%+.1f', '%.1f', 'marker position in the camera frame';
  'Orientation', 'oriEst', 'oriGt', 'ori', {'About marker X (tilt)', 'About marker Y (tilt)', ...
                 'About marker Z (in-plane)'}, ...
                 'deg', '%+.2f', '%.2f', 'marker orientation relative to the reference pose'};
for q = 1:size(KINDS, 1)
[qName, fEst, fGt, sPre, axTS, unit, fmtM, fmtS, qTitle] = KINDS{q, :};
TSCOL  = [0.00 0.45 0.74; 0.85 0.33 0.10];   % ArUco dots: baseline blue, dark orange
GTCOL  = [0.00 0.00 0.00; 0.13 0.55 0.13];   % ground truth: baseline black, dark green
GTSTY  = {'-', '--'};
for a = 1:numel(tsAngles)
    sun = tsAngles{a};
    if ~any(strcmp(S.sun, sun)), continue; end
    nC = numel(supsP);
    figs(end+1) = figure('Name', sprintf('%s over time, %s', qName, angLabel(sun)), ...
                         'Color', 'w', 'Position', [40 40 400 * nC + 120, 820]); %#ok<SAGROW>
    tl = tiledlayout(3, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    axGrid  = gobjects(3, nC);
    hasData = false(3, nC);
    for c = 1:nC
        idx = zeros(1, numel(SETS));            % trial row per set (0 = none)
        for s = 1:numel(SETS)
            ii = find(strcmp(S.set, SETS{s}) & strcmp(S.sun, sun) & strcmp(S.sup, supsP{c}), 1);
            if ~isempty(ii), idx(s) = ii; end
        end
        for k = 1:3
            ax = nexttile(tl, (k - 1) * nC + c);  axGrid(k, c) = ax;
            hold(ax, 'on'); grid(ax, 'on');
            if k == 1, title(ax, sprintf('sup light: %s', supsP{c})); end
            if c == 1, ylabel(ax, sprintf('%s (%s)', axTS{k}, unit)); end
            if k == 3, xlabel(ax, 'Time (s)'); end

            detTxt = {};  errLine = 0;
            for s = 1:numel(SETS)
                i = idx(s);
                if i == 0
                    detTxt{end+1} = sprintf('%s: no trial', SETS{s}); %#ok<SAGROW>
                    continue;
                end
                ts = TS{i};
                if isempty(ts) || isempty(ts.t)
                    if S.nDetected(i) == 0
                        detTxt{end+1} = sprintf('%s: no detections (0 %%)', SETS{s}); %#ok<SAGROW>
                    else
                        detTxt{end+1} = sprintf('%s: no pose data', SETS{s}); %#ok<SAGROW>
                    end
                    continue;
                end
                hasData(k, c) = true;
                plot(ax, ts.t, ts.(fGt)(:, k), GTSTY{s}, 'Color', GTCOL(s, :), 'LineWidth', 1.4, ...
                     'DisplayName', sprintf('%s ground truth', SETS{s}));
                plot(ax, ts.t, ts.(fEst)(:, k), '.', 'Color', TSCOL(s, :), 'MarkerSize', 8, ...
                     'DisplayName', sprintf('%s ArUco', SETS{s}));
                detTxt{end+1} = sprintf('%s: det %.0f %%', SETS{s}, S.detRate(i)); %#ok<SAGROW>

                % Mean error (ArUco - ground truth) on this axis, same value as the tables
                mu = S.([sPre ax3(k) '_mean'])(i);
                sd = S.([sPre ax3(k) '_std'])(i);
                if isnan(mu)
                    errTxt = sprintf('%s mean error: n/a', SETS{s});
                else
                    errTxt = sprintf(['%s mean error ' fmtM ' %s (std ' fmtS ')'], ...
                                     SETS{s}, mu, unit, sd);
                end
                text(ax, 0.98, 0.96 - 0.11 * errLine, errTxt, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
                     'FontSize', 8, 'FontWeight', 'bold', 'Color', TSCOL(s, :), ...
                     'BackgroundColor', 'w', 'Margin', 1);
                errLine = errLine + 1;
            end
            if ~hasData(k, c)
                noData(ax, strjoin(detTxt, newline));
                continue;
            end
            if k == 1
                text(ax, 0.02, 0.96, strjoin(detTxt, newline), 'Units', 'normalized', ...
                     'VerticalAlignment', 'top', 'FontSize', 8, 'BackgroundColor', 'w', ...
                     'Margin', 1);
            end
        end
    end
    % Legend once (first tile with data)
    [kk, cc] = find(hasData, 1);
    if ~isempty(kk), legend(axGrid(kk, cc), 'Location', 'southeast', 'FontSize', 7); end
    % Same y-range across supplementary levels for each axis, so the levels compare directly
    for k = 1:3
        h = axGrid(k, hasData(k, :));
        if numel(h) > 1, linkaxes(h, 'y'); end
    end
    title(tl, sprintf(['Sun %s: ArUco (dots) vs ground truth (lines), baseline and dark, ' ...
                       '%s. Mean error = ArUco - ground truth'], angLabel(sun), qTitle));
end
end

%% ===================== OBJECTIVE ANALYSES (1 to 5) ========================
% Uses only the per-trial results in S. Every change is judged against the noise
% floor from the repeated reference trials (objective 4), so a difference smaller
% than run-to-run variation is reported as "within noise".
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
outDir = fullfile(ROOTS{1}, 'analysis_results');
if SAVE_RESULTS && ~exist(outDir, 'dir'), mkdir(outDir); end
saveRows = @(file, names, rows) SAVE_RESULTS && writeRows(fullfile(outDir, file), names, rows);

% ---- Objective 4 (repeatability): noise floor --------------------------------
NF = noiseFloor(S, MIN_NOISE, NOISE_K);
[nfNames, nfRows] = noiseFloorRows(NF);
printRows(sprintf(['OBJECTIVE 4: REPEATABILITY of the reference condition (%d repeats). ' ...
                   'Changes smaller than "threshold" are treated as noise'], NF.nRepeats), ...
          nfNames, nfRows);
saveRows('objective4_repeatability.csv', nfNames, nfRows);
repRows = repeatSpread(S);                                % every repeated condition
if ~isempty(repRows)
    printRows('OBJECTIVE 4: SPREAD OF EVERY REPEATED CONDITION', ...
              {'set', 'sun', 'sup', 'n', 'det_sd_pct', 'pos3D_rmse_sd_mm', 'ori_rmse_sd_deg'}, repRows);
end

% ---- changes vs the ideal reference and vs no supplementary light -------------
DM = {'detRate', 'pos3D_rmse', 'pos3D_std', 'pos3D_mean', 'posX_rmse', 'posY_rmse', ...
      'posZ_rmse', 'oriTot_rmse', 'oriTot_std', 'oriTot_max', 'flipRate'};
S = addDeltas(S, DM);                         % adds dRef_<m> and dSup_<m> columns
S.failMode = classifyTrials(S, NF, HARD_THRESHOLD);
S.supLux   = supLux(S, LUX_SUP, SUP_LEVELS);

% ---- Objective 1: baseline degradation ---------------------------------------
[o1Names, o1Rows] = objective1Rows(S, SETS, angOrder);
printRows(['OBJECTIVE 1: DEGRADATION WITHOUT SUPPLEMENTARY LIGHT ' ...
           '(d_ = change vs Ambient reference: no sun, no supplementary light)'], o1Names, o1Rows);
saveRows('objective1_degradation.csv', o1Names, o1Rows);

figs(end+1) = figure('Name', 'Objective 1 degradation', 'Color', 'w', 'Position', [60 40 1500 760]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
o1m = {'detRate', 'Detection rate (%)'; 'pos3D_rmse', '3D position RMSE (mm)';
       'pos3D_std', '3D position std (mm)'; 'oriTot_rmse', 'Orientation RMSE (deg)'};
for c = 1:size(o1m, 1)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    for s = 1:numel(SETS)
        y = arrayfun(@(a) getMetric(S, SETS{s}, angOrder{a}, 'none', o1m{c,1}), 1:nA);
        plot(ax, 1:nA, y, SETMK{s}, 'Color', SETCOL(s,:), 'LineWidth', 1.5, ...
             'MarkerFaceColor', SETCOL(s,:), 'DisplayName', SETS{s});
    end
    refv = getMetric(S, 'baseline', 'none', 'none', o1m{c,1});
    if ~isnan(refv)
        h = yline(ax, refv, '--', 'reference'); h.HandleVisibility = 'off';
    end
    xticks(ax, 1:nA); xticklabels(ax, angLabels); xlim(ax, [0.5 nA + 0.5]);
    xlabel(ax, 'Sun angle'); title(ax, o1m{c,2});
    if c == 1, ylim(ax, [0 105]); legend(ax, 'Location', 'best'); end
end
title(tl, 'Objective 1: performance without supplementary light (dashed = ideal reference)');
% ---- Objective 1: sun-angle degradation, no lamp vs High lamp ----------------
% Each condition is compared with the same environment and lamp level with no sun,
% so the High-lamp rows show the sun's effect with detection guaranteed.
F1rows = {'baseline', 'none', 'Ambient, no lamp';
          'baseline', 'high', 'Ambient, High lamp';
          'dark',     'none', 'Dark, no lamp';
          'dark',     'high', 'Dark, High lamp'};
d1 = {'pos3D_rmse',  '\Delta 3D position RMSE (mm)';
      'pos3D_std',   '\Delta 3D position std (mm)';
      'oriTot_rmse', '\Delta orientation RMSE (deg)';
      'flipRate',    '\Delta orientation jumps (pp)'};
figs(end+1) = figure('Name', 'Objective 1 sun effect', 'Color', 'w', ...
                     'Position', [60 40 1700 950]);
tl = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl, [1 4]);
[codes, words] = failureGridLamp(S, F1rows, angOrder, NF, HARD_THRESHOLD);
imagesc(ax, codes, 'AlphaData', ~isnan(codes));
set(ax, 'Color', [0.88 0.88 0.88]);
colormap(ax, [0.70 0.15 0.15; 0.93 0.55 0.20; 0.97 0.85 0.40; 0.35 0.62 0.42; 0.62 0.70 0.80]);
caxis(ax, [0.5 5.5]);
for r = 1:size(codes, 1)
    for c = 1:size(codes, 2)
        if ~isnan(codes(r, c))
            text(ax, c, r, words{r, c}, 'HorizontalAlignment', 'center', ...
                 'FontWeight', 'bold', 'FontSize', 9);
        end
    end
end
xticks(ax, 1:nA); xticklabels(ax, angLabels);
yticks(ax, 1:size(F1rows, 1)); yticklabels(ax, F1rows(:, 3));
xlabel(ax, 'Sun angle');
title(ax, 'Effect of the sun vs the same environment and lamp level with no sun');

hiSets = {'baseline', 'dark'};
for m = 1:size(d1, 1)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    Y = nan(nA, numel(hiSets));
    for s = 1:numel(hiSets)
        ref = getMetric(S, hiSets{s}, 'none', 'high', d1{m,1});
        Y(:, s) = arrayfun(@(a) getMetric(S, hiSets{s}, angOrder{a}, 'high', d1{m,1}) - ref, 1:nA)';
    end
    groupedBar(ax, angLabels, Y, {'Ambient, High', 'Dark, High'}, SETCOL);
    yline(ax, 0, 'k-', 'HandleVisibility', 'off');
    title(ax, d1{m,2}); xlabel(ax, 'Sun angle');
    if m == 1, legend(ax, 'Location', 'best'); end
end
title(tl, ['Objective 1: degradation caused by the sun angle ' ...
           '(bars: High lamp, change vs High lamp with no sun)']);
% ---- Objective 3: mitigation with supplementary light --------------------------
[o3Names, o3Rows] = objective3Rows(S, SETS, angOrder, SUP_LEVELS, NF);
printRows(['OBJECTIVE 3: WITH SUPPLEMENTARY LIGHT (d_ = change vs no supplementary ' ...
           'light at the same sun angle; dref_ = change vs the ideal reference)'], o3Names, o3Rows);
saveRows('objective3_mitigation.csv', o3Names, o3Rows);
if all(isnan(S.procMean_ms))
    fprintf(['\n  Processing time: not recorded in the bags. Run time_aruco_pipeline.py ' ...
             'to measure it offline.\n']);
else
    fprintf('\n  Processing time: mean %.2f ms, max %.2f ms per frame (all trials)\n', ...
            mean(S.procMean_ms, 'omitnan'), max(S.procMax_ms, [], 'omitnan'));
end

if ~isempty(supsP)
    % Accuracy and stability change vs no supplementary light (red = worse).
    figs(end+1) = figure('Name', 'Objective 3 accuracy change', 'Color', 'w', ...
                         'Position', [60 40 1700 760]);
    tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    o3m = {'dSup_pos3D_rmse', 'change in 3D position RMSE (mm)';
           'dSup_pos3D_std',  'change in 3D position std (mm)';
           'dSup_flipRate',   'change in orientation jumps (pp)'};
    for s = 1:2
        for c = 1:size(o3m, 1)
            ax = nexttile(tl);
            M = nan(numel(supsP), nA);
            for r = 1:numel(supsP)
                for a = 1:nA
                    M(r, a) = getMetric(S, SETS{s}, angOrder{a}, supsP{r}, o3m{c,1});
                end
            end
            m = max(abs(M(:)));
            if isempty(m) || isnan(m) || m == 0, m = 1; end
            drawHeat(ax, M, angLabels, supsP, [-m m], flipud(divMap(256)), '%+.1f');
            title(ax, sprintf('%s: %s', upper(SETS{s}), o3m{c,2}));
        end
    end
    title(tl, ['Objective 3: accuracy and stability change with supplementary light ' ...
               '(red = worse, blue = better; blank = no detections)']);

    % Detection vs supplementary illuminance, one line per sun angle.
    figs(end+1) = figure('Name', 'Objective 3 detection vs lux', 'Color', 'w', ...
                         'Position', [80 60 1400 520]);
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    cols = lines(nA);
    for s = 1:2
        ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
        lux = LUX_SUP.(SETS{s});
        ok = ~isnan(lux(1:min(numel(lux), numel(SUP_LEVELS))));
        if sum(ok) < 2
            noData(ax, sprintf('%s: add measured lux values to LUX_SUP', SETS{s}));
            continue;
        end
        for a = 1:nA
            y = arrayfun(@(r) getMetric(S, SETS{s}, angOrder{a}, SUP_LEVELS{r}, 'detRate'), ...
                         find(ok));
            plot(ax, lux(ok), y, 'o-', 'Color', cols(a,:), 'LineWidth', 1.4, ...
                 'MarkerFaceColor', cols(a,:), 'DisplayName', angLabels{a});
        end
        hline(ax, HARD_THRESHOLD);
        ylim(ax, [0 105]); xlabel(ax, 'Illuminance at the marker from supplementary light (lux)');
        ylabel(ax, 'Detection rate (%)'); title(ax, upper(SETS{s}));
        legend(ax, 'Location', 'southeast');
    end
    title(tl, 'Objective 3: detection vs supplementary illuminance, per sun angle');
end

% ---- Objective 2: supplementary light design ---------------------------------
[o2Names, o2Rows, reqNames, reqRows, interferenceOK] = objective2Rows( ...
    S, SETS, SUP_LEVELS, NF, HARD_THRESHOLD, INTERFERENCE_TOL_PCT, LUX_SUP);
printRows(['OBJECTIVE 2: SUPPLEMENTARY LIGHT ALONE (no sun), ' ...
           'd_ = change vs no supplementary light'], o2Names, o2Rows);
printRows('OBJECTIVE 2: DESIGN REQUIREMENTS', reqNames, reqRows);
saveRows('objective2_lamp_only.csv', o2Names, o2Rows);
saveRows('objective2_requirements.csv', reqNames, reqRows);

% Lamp-only change vs lamp off (no sun) + OptiTrack dropouts
lv = supsP(~strcmp(supsP, 'none'));                % low / med / high
d2 = {'pos3D_rmse',  '\Delta 3D position RMSE (mm)';
      'pos3D_std',   '\Delta 3D position std (mm)';
      'oriTot_max',  '\Delta max orientation error (deg)';
      'flipRate',    '\Delta orientation jumps (pp)'};
figs(end+1) = figure('Name', 'Objective 2 lamp change', 'Color', 'w', ...
                     'Position', [60 40 1800 460]);
tl = tiledlayout(1, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
legTxt = {'Ambient (vs lamp off)', 'Dark (vs lamp off)'};
for m = 1:size(d2, 1)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    Y = nan(numel(lv), numel(SETS));
    for s = 1:numel(SETS)
        for k = 1:numel(lv)
            v = getMetric(S, SETS{s}, 'none', lv{k}, ['dSup_' d2{m,1}]);
            if isnan(v)                            % lamp-off had no poses (Dark)
                v = getMetric(S, SETS{s}, 'none', lv{k}, ['dRef_' d2{m,1}]);
                if ~isnan(v), legTxt{s} = sprintf('%s (vs ideal reference)', ...
                                                  regexprep(legTxt{s}, ' \(.*\)', '')); end
            end
            Y(k, s) = v;
        end
    end
    groupedBar(ax, lv, Y, legTxt, SETCOL);
    yline(ax, 0, 'k-', 'HandleVisibility', 'off');
    title(ax, d2{m,2}); xlabel(ax, 'Supplementary light (no sun)');
    if m == 1, legend(ax, 'Location', 'best'); end
end

% OptiTrack dropout with the lamp off vs on (Ambient trials, all sun angles)
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
dp = arrayfun(@(k) mean(S.mocapDropPct(strcmp(S.set, 'baseline') & ...
              strcmp(S.sup, supsP{k})), 'omitnan'), 1:numel(supsP));
bar(ax, dp, 'FaceColor', SETCOL(1,:));
for k = 1:numel(dp)                                % label every bar, even at 0 %
    if ~isnan(dp(k))
        text(ax, k, dp(k), sprintf('%.1f%%', dp(k)), 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
    end
end
xticks(ax, 1:numel(supsP)); xticklabels(ax, supsP);
if ~isnan(dp(1))
    yline(ax, dp(1) + INTERFERENCE_TOL_PCT, ':', 'tolerance', 'HandleVisibility', 'off');
    ylim(ax, [0 max([dp, dp(1) + INTERFERENCE_TOL_PCT]) * 1.3 + eps]);
end
ylabel(ax, '% OptiTrack samples dropped'); xlabel(ax, 'Supplementary light');
title(ax, 'OptiTrack interference (Ambient)');
title(tl, 'Objective 2: effect of the lamp on its own (change vs lamp off, no sun)');

% ---- Objective 5: suitability for Yandiwanba ----------------------------------
[o5Names, o5Rows, verdict] = objective5Rows(S, SETS, angOrder, SUP_LEVELS, NF, ...
    HARD_THRESHOLD, ACCURACY_TARGET_MM, ACCURACY_TARGET_DEG, FRAME_RATE_HZ, ...
    LUX_SUP, SUN_LUX, interferenceOK);
printRows(['OBJECTIVE 5: SUITABILITY (lowest supplementary level reaching the ' ...
           'availability threshold at each sun angle)'], o5Names, o5Rows);
fprintf('\n');
for k = 1:numel(verdict), fprintf('  %s\n', verdict{k}); end
saveRows('objective5_suitability.csv', o5Names, o5Rows);



%% ============================== SAVE =====================================
if ~exist('figs', 'var'), figs = gobjects(0); end   % lets this section run on its own
if SAVE_RESULTS
    outDir = fullfile(ROOTS{1}, 'analysis_results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    writetable(S, fullfile(outDir, 'trial_summary.csv'));
    scT = table(SC.label(:), SC.cls(:), SC.rec(:), SC.best(:), ...
                'VariableNames', {'scenario', 'class', 'recovered_by', 'best_sup'});
    for r = 1:numel(supsP)
        scT.(['det_' supsP{r}])        = SC.det(:, r);
        scT.(['posX_rmse_' supsP{r}])  = SC.px(:, r);
        scT.(['posY_rmse_' supsP{r}])  = SC.py(:, r);
        scT.(['posZ_rmse_' supsP{r}])  = SC.pz(:, r);
        scT.(['ori_rmse_' supsP{r}])   = SC.ori(:, r);
    end
    writetable(scT, fullfile(outDir, 'scenarios.csv'));
    for k = 1:numel(figs)
        name = regexprep(lower(figs(k).Name), '[^a-z0-9]+', '_');
        exportgraphics(figs(k), fullfile(outDir, sprintf('%02d_%s.png', k, name)), ...
                       'Resolution', 200);
    end
    fprintf('\nSaved CSVs and %d figures to %s\n', numel(figs), outDir);
end

%% ======================== LOCAL FUNCTIONS ================================

function T = autoTrials(root, gtBag)
% Build the trial list from bag folder names, e.g.
%   baseline_sun30_suplow, dark_base_supnone, dark_0deg_high, baseline_nosun
    d = dir(root);
    d = d([d.isdir] & ~startsWith({d.name}, '.'));
    T = cell(0, 4);
    for k = 1:numel(d)
        nm = d(k).name;
        if ~isfile(fullfile(root, nm, 'metadata.yaml')) || strcmpi(nm, gtBag)
            continue;
        end
        s = lower(nm);
        if contains(s, 'dark')
            set = 'dark';
        elseif contains(s, 'base')
            set = 'baseline';
        else
            fprintf('  skipped %s (no "base" or "dark" in the name)\n', nm);
            continue;
        end
        r = regexprep(s, '(baseline|dark)', '');      % leave the condition part
        sun = 'none';
        if ~isempty(regexp(r, 'no_?sun', 'once'))
            sun = 'none';
        else
            tok = regexp(r, '(?:sun|angle|ang)_?(\d+)', 'tokens', 'once');
            if isempty(tok), tok = regexp(r, '(\d+)_?deg', 'tokens', 'once'); end
            if ~isempty(tok)
                sun = tok{1};
            elseif ~isempty(regexp(r, '(^|_)base', 'once'))
                sun = 'none';                      % "base" = base lighting, no sun
            else
                tok = regexp(r, '(?:^|_)(\d+)(?:_|$)', 'tokens', 'once');
                if ~isempty(tok), sun = tok{1}; end
            end
        end
        sup = 'none';
        tok = regexp(r, 'sup\w*?_?(none|off|low|medium|med|mid|high)', 'tokens', 'once');
        if isempty(tok)
            tok = regexp(r, '(?:^|_)(low|medium|med|mid|high)(?:_|$|\d)', 'tokens', 'once');
        end
        if ~isempty(tok), sup = tok{1}; end
        T(end+1, :) = {nm, set, sun, sup}; %#ok<AGROW>
    end
end

function p = bagPath(roots, name)
% Full path of a bag folder, searched for in every root folder.
    for r = 1:numel(roots)
        p = fullfile(roots{r}, name);
        if isfolder(p), return; end
    end
    p = fullfile(roots{1}, name);      % not found: analyseTrial reports it
end

function T = normaliseTrials(T, gtBag)
    keep = true(size(T, 1), 1);
    for i = 1:size(T, 1)
        T{i,2} = lower(strtrim(T{i,2}));
        if strcmp(T{i,2}, 'base'), T{i,2} = 'baseline'; end
        T{i,3} = normAngle(T{i,3});
        T{i,4} = normSup(T{i,4});
        if ~ismember(T{i,2}, {'baseline', 'dark'})
            fprintf('  dropped %s: set must be ''baseline'' or ''dark''\n', T{i,1});
            keep(i) = false;
        end
        if strcmpi(T{i,1}, gtBag), keep(i) = false; end
    end
    T = T(keep, :);
end

function a = normAngle(a)
    if isnumeric(a)
        if isempty(a) || isnan(a), a = 'none'; else, a = sprintf('%g', a); end
        return;
    end
    a = lower(strtrim(char(a)));
    a = regexprep(a, '(deg|degrees|°)$', '');
    if isempty(a) || any(strcmp(a, {'nosun', 'no sun', 'no_sun', 'nan', 'off', 'base'}))
        a = 'none';
    elseif ~isnan(str2double(a))
        a = sprintf('%g', str2double(a));    % '030' -> '30'
    end
end

function s = normSup(s)
    s = lower(strtrim(char(s)));
    switch s
        case {'', 'none', 'off', 'no', 'nosup', '0'}, s = 'none';
        case {'med', 'medium', 'mid'},                s = 'med';
        case {'low', 'high'}
        otherwise
            warning('Unknown supplementary light level "%s"', s);
    end
end

function order = angleOrder(cats, userOrder)
    cats = unique(cats(:)', 'stable');
    if ~isempty(userOrder)
        userOrder = cellfun(@normAngle, userOrder(:)', 'UniformOutput', false);
        order = [userOrder(ismember(userOrder, cats)), setdiff(cats, userOrder, 'stable')];
        return;
    end
    num = str2double(cats);
    isNone = strcmp(cats, 'none');
    txt = sort(cats(isnan(num) & ~isNone));
    nums = cats(~isnan(num));
    [~, o] = sort(num(~isnan(num)));
    order = [cats(isNone), txt, nums(o)];
end

function s = angLabel(a)
    if strcmp(a, 'none')
        s = 'no sun (base)';
    elseif ~isnan(str2double(a))
        s = [a char(176)];
    else
        s = a;
    end
end

% ---------------------------------------------------------------------------
function ref = loadReference(folder, BC_T_C, BM_T_M, P)
% Static camera and ground-truth marker poses (OptiTrack world) from the
% reference bag, built exactly like quick_analysis.py:
%   camera optical pose   O_T_C   = O_T_BC * BC_T_C
%   ground-truth marker   O_T_Mgt = O_T_BM * BM_T_M   (known offset applied here,
%                                                      the bag does not contain it)
    if ~isfolder(folder)
        error('Reference ground-truth bag not found: %s', folder);
    end
    bag = ros2bagreader(folder);
    CB = readPoses(bag, P.topicCamRB);
    MB = readPoses(bag, P.topicMarkerRB);
    if isempty(CB) || isempty(MB)
        error(['Reference bag %s has no OptiTrack camera/marker rigid-body data, ' ...
               'so it cannot be used as ground truth.'], folder);
    end
    keepC = ~inWindows(CB(:,1), findDropouts(CB, P.minFrozenSamples, P.jumpThreshold), P.dropoutMargin);
    keepM = ~inWindows(MB(:,1), findDropouts(MB, P.minFrozenSamples, P.jumpThreshold), P.dropoutMargin);
    if ~any(keepC) || ~any(keepM)
        error('No usable OptiTrack samples in the reference bag %s.', folder);
    end
    ref.O_T_BC  = medianPose(CB(keepC, :));
    ref.O_T_C   = ref.O_T_BC * BC_T_C;
    ref.O_T_Mgt = medianPose(MB(keepM, :)) * BM_T_M;
    ref.C_T_M   = ref.O_T_C \ ref.O_T_Mgt;            % for the printed summary only
    ref.source  = 'OptiTrack';
    ref.n       = sum(keepM);
    ref.spread_mm = 1000 * max([std(MB(keepM, 2:4), 0, 1), std(CB(keepC, 2:4), 0, 1)]);
end

% ---------------------------------------------------------------------------
function [R, ts] = analyseTrial(folder, mode, ref, BC_T_C, BM_T_M, P)
% ts: per-frame marker position in the camera frame (mm) for the time-series plots
%     t (s from first frame), est (ArUco), gt (ground truth, NaN where unavailable)
    R = emptyResult();
    ts = [];
    R.gtMode = mode;
    if ~isfolder(folder), error('bag folder not found: %s', folder); end
    bag = ros2bagreader(folder);

    % ---- detection rate --------------------------------------------------
    det = readInts(bag, P.topicDetected);
    if ~isempty(det)
        R.nFrames = numel(det);
        R.nDetected = sum(det > 0);
    else
        tf = readInts(bag, P.topicTotal);
        df = readInts(bag, P.topicDetFrames);
        if ~isempty(tf) && ~isempty(df)
            R.nFrames = max(tf);
            R.nDetected = max(df);
        end
    end
    R.detRate = 100 * R.nDetected / R.nFrames;
    R.nDetectedAny = R.nDetected;          % any ID, before the ID check below
    R.detRateAny   = R.detRate;

    % ---- processing time (only if the detector published it) -----------------
    pt = readInts(bag, P.topicProc);
    if ~isempty(pt)
        R.procMean_ms = mean(pt);
        R.procMax_ms  = max(pt);
    end

    % ---- no detections: nothing to compare against ground truth -------------
    if R.nDetected == 0
        R.note = 'no detections (0 %): not compared to ground truth';
        return;
    end
    Araw = readPosesRaw(bag, P.topicAruco);

    % ---- marker ID check (objective 1: misidentification) --------------------
    % The detector publishes one /aruco/id message per /aruco/pose message, in the
    % same order, so the k-th ID belongs to the k-th pose. Poses of any other ID are
    % removed, and detection then counts only frames with the expected marker.
    ids = readInts(bag, P.topicId);
    if ~isempty(ids)
        R.nWrongID = sum(ids ~= P.markerId);
        if numel(ids) == size(Araw, 1)
            Araw = Araw(ids == P.markerId, :);
            R.idChecked = 1;
            nGood = numel(unique(Araw(:,1)));          % frames with the right marker
            if ~isnan(R.nFrames)
                R.nDetected = min(nGood, R.nFrames);
                R.detRate   = 100 * R.nDetected / R.nFrames;
            end
        else
            R.note = 'ID and pose counts differ: wrong IDs counted but not removed';
        end
    end

    A = cleanPoses(Araw);
    R.nPose = size(A, 1);
    if isempty(A)
        R.note = 'no ArUco poses of the expected marker: not compared to ground truth';
        return;
    end
    A(:,1) = A(:,1) + P.timeOffset;
    n = size(A, 1);
    ts = struct('t', A(:,1) - A(1,1), 'est', 1000 * A(:,2:4), 'gt', nan(n, 3), ...
                'oriEst', nan(n, 3), 'oriGt', nan(n, 3));

    % ---- camera and ground-truth marker poses in the OptiTrack world ---------
    % Same method as quick_analysis.py.
    O_T_C   = zeros(4, 4, n);   % camera optical frame
    O_T_Mgt = zeros(4, 4, n);   % ground-truth ArUco centre (known offset applied)
    valid = true(n, 1);
    switch mode
        case 'own'
            CB = readPoses(bag, P.topicCamRB);
            MB = readPoses(bag, P.topicMarkerRB);
            if isempty(CB) || isempty(MB)
                R.note = 'no OptiTrack data in this bag: not compared';
                return;
            end
            R.gtSource = 'own OptiTrack';
            W = [findDropouts(MB, P.minFrozenSamples, P.jumpThreshold);
                 findDropouts(CB, P.minFrozenSamples, P.jumpThreshold)];
            valid = A(:,1) >= max(CB(1,1), MB(1,1)) & A(:,1) <= min(CB(end,1), MB(end,1));
            valid = valid & ~inWindows(A(:,1), W, P.dropoutMargin);
            for k = 1:n
                O_T_C(:,:,k)   = interpPose(CB, A(k,1)) * BC_T_C;
                O_T_Mgt(:,:,k) = interpPose(MB, A(k,1)) * BM_T_M;
            end
            R.nDropouts    = size(W, 1);
            R.mocapDropPct = 100 * mean(inWindows(MB(:,1), W, 0));
        case 'static'
            % Own OptiTrack data, reduced to one median pose (static marker only).
            CB = readPoses(bag, P.topicCamRB);
            MB = readPoses(bag, P.topicMarkerRB);
            if isempty(CB) || isempty(MB)
                R.note = 'no OptiTrack data in this bag: not compared';
                return;
            end
            R.gtSource = 'own OptiTrack (static median)';
            WC = findDropouts(CB, P.minFrozenSamples, P.jumpThreshold);
            WM = findDropouts(MB, P.minFrozenSamples, P.jumpThreshold);
            keepC = ~inWindows(CB(:,1), WC, P.dropoutMargin);
            keepM = ~inWindows(MB(:,1), WM, P.dropoutMargin);
            if ~any(keepC) || ~any(keepM)
                R.note = 'no usable OptiTrack samples: not compared';
                return;
            end
            O_T_C   = repmat(medianPose(CB(keepC, :)) * BC_T_C, 1, 1, n);
            O_T_Mgt = repmat(medianPose(MB(keepM, :)) * BM_T_M, 1, 1, n);
            R.nDropouts    = size(WM, 1);
            R.mocapDropPct = 100 * mean(~keepM);
        case 'reference'
            if isempty(ref), error('reference ground truth not loaded'); end
            R.gtSource = ['reference: ' ref.source];
            O_T_C   = repmat(ref.O_T_C,   1, 1, n);
            O_T_Mgt = repmat(ref.O_T_Mgt, 1, 1, n);
            % Ground truth = GT_BAG's OptiTrack. The dark bag's own camera pose (if
            % recorded) is only used to warn if the camera moved since GT_BAG.
            CB = readPoses(bag, P.topicCamRB);
            if ~isempty(CB)
                Tc = medianPose(CB);
                R.camMove_mm  = 1000 * norm(Tc(1:3,4) - ref.O_T_BC(1:3,4));
                R.camMove_deg = rad2deg(norm(rotmToRotvec(ref.O_T_BC(1:3,1:3)' * Tc(1:3,1:3))));
            end
        otherwise
            error('GT mode must be ''own'', ''static'' or ''reference''');
    end
    R.nExcluded = sum(~valid);
    R.nValid = sum(valid);

    % ---- errors (quick_analysis.py method) ------------------------------------
    %   est   = O_T_C * C_T_M              ArUco estimate in the world
    %   e     = R_OC' * (t_est - t_gt)     position error in camera axes (mm)
    %   R_err = R_gt' * R_est              orientation error about marker axes (deg)
    err = zeros(n, 3);
    ori = zeros(n, 3);
    RgtC = zeros(3, 3, n);
    for k = 1:n
        est = O_T_C(:,:,k) * poseToT(A(k,2:8));
        gt  = O_T_Mgt(:,:,k);
        e   = est(1:3,4) - gt(1:3,4);
        err(k,:) = 1000 * (O_T_C(1:3,1:3,k)' * e)';
        ori(k,:) = rad2deg(rotmToRotvec(gt(1:3,1:3)' * est(1:3,1:3)))';
        C_T_Mgt  = O_T_C(:,:,k) \ gt;                 % ground truth in the camera frame
        ts.gt(k,:) = 1000 * C_T_Mgt(1:3,4)';
        RgtC(:,:,k) = C_T_Mgt(1:3,1:3);
    end
    ts.gt(~valid, :) = NaN;                           % mocap dropouts leave gaps

    % Orientation series for plotting: rotation of the marker (ArUco and ground
    % truth, both in the camera frame) relative to one reference orientation R0,
    % as angles about the marker X / Y / Z axes (deg). R0 is the reference-bag
    % ground truth when loaded, so baseline and dark share the same zero.
    if ~isempty(ref)
        R0 = ref.C_T_M(1:3,1:3);
    else
        vk = find(valid, 1); if isempty(vk), vk = 1; end
        R0 = RgtC(:,:,vk);
    end
    for k = 1:n
        Rest = poseToT(A(k,2:8));
        ts.oriEst(k,:) = rad2deg(rotmToRotvec(R0' * Rest(1:3,1:3)))';
        ts.oriGt(k,:)  = rad2deg(rotmToRotvec(R0' * RgtC(:,:,k)))';
    end
    ts.oriGt(~valid, :) = NaN;

    outl = false(n, 1);
    if P.removeOutliers && sum(valid) >= P.outlierWindow
        outl = findOutliers(ori, valid, P.outlierWindow, P.kMAD, P.minOriOutlier) | ...
               findOutliers(err, valid, P.outlierWindow, P.kMAD, P.minPosOutlier);
        outl = outl & valid;
    end
    used = valid & ~outl;
    R.nOutlier = sum(outl);
    R.nUsed = sum(used);

    % ---- orientation jumps / solution flips (objective 3) ---------------------
    % Same robust test as the outlier filter, but always computed and never used
    % to remove frames: the percentage of valid frames whose orientation error
    % departs from its running median (switches between the two planar PnP
    % solutions show up here).
    if sum(valid) >= P.outlierWindow
        flip = findOutliers(ori, valid, P.outlierWindow, P.kMAD, P.minOriOutlier);
        R.nFlip    = sum(flip);
        R.flipRate = 100 * R.nFlip / sum(valid);
    end
    if R.nUsed == 0
        R.note = 'no usable frames (check clocks / OptiTrack tracking)';
    end

    % ---- statistics ---------------------------------------------------------
    E = err(used, :);  O = ori(used, :);
    oT = sqrt(sum(O.^2, 2));
    sp = errStats(E);  so = errStats(O);
    ax = 'XYZ';  st = {'mean', 'std', 'rmse'};
    for a = 1:3
        for q = 1:3
            R.(['pos' ax(a) '_' st{q}]) = sp(q, a);
            R.(['ori' ax(a) '_' st{q}]) = so(q, a);
        end
    end
    sT = errStats(oT);
    for q = 1:3
        R.(['oriTot_' st{q}]) = sT(q);
    end
    R.oriTot_max = safeMax(oT);

    % 3D position error: bias = length of the mean error vector, std = combined
    % spread of the three axes, RMSE = RMS of the 3D error length.
    if ~isempty(E)
        R.pos3D_mean = norm(mean(E, 1));
        R.pos3D_std  = sqrt(sum(std(E, 0, 1).^2));
        R.pos3D_rmse = sqrt(mean(sum(E.^2, 2)));
    end
end

function R = emptyResult()
    R = struct('bag', '', 'set', '', 'sun', '', 'sup', '', 'gtMode', '', 'gtSource', '', ...
               'nFrames', NaN, 'nDetected', NaN, 'detRate', NaN, 'nPose', 0, ...
               'nValid', 0, 'nExcluded', 0, 'nOutlier', 0, 'nUsed', 0);
    ax = 'XYZ';  st = {'mean', 'std', 'rmse'};
    for a = 1:3
        for q = 1:3, R.(['pos' ax(a) '_' st{q}]) = NaN; end
    end
    for a = 1:3
        for q = 1:3, R.(['ori' ax(a) '_' st{q}]) = NaN; end
    end
    for q = {'mean', 'std', 'rmse', 'max'}, R.(['oriTot_' q{1}]) = NaN; end
    R.pos3D_mean = NaN;  R.pos3D_std = NaN;  R.pos3D_rmse = NaN;
    R.nDetectedAny = NaN;  R.detRateAny = NaN;
    R.nWrongID = NaN;  R.idChecked = 0;
    R.nFlip = NaN;  R.flipRate = NaN;
    R.nDropouts = NaN;  R.mocapDropPct = NaN;
    R.procMean_ms = NaN;  R.procMax_ms = NaN;
    R.camMove_mm = NaN;
    R.camMove_deg = NaN;
    R.note = '';
end

function S = resultsToTable(results)
    F = fieldnames(results{1});
    S = table();
    for f = 1:numel(F)
        vals = cellfun(@(r) r.(F{f}), results, 'UniformOutput', false);
        if all(cellfun(@(v) isnumeric(v) && isscalar(v), vals))
            S.(F{f}) = cell2mat(vals(:));
        else
            S.(F{f}) = vals(:);
        end
    end
end

% ---------------------------------------------------------------------------
function v = getMetric(S, set, sun, sup, field)
% Mean of a metric over all trials (repeats) of one condition; NaN if none.
    m = strcmp(S.set, set) & strcmp(S.sun, sun) & strcmp(S.sup, sup);
    if ~any(m), v = NaN; else, v = mean(S.(field)(m), 'omitnan'); end
end

function idx = sortedIdx(S, set, angOrder, sups)
    idx = find(strcmp(S.set, set));
    if isempty(idx), return; end
    [~, ia] = ismember(S.sun(idx), angOrder);
    [~, is] = ismember(S.sup(idx), sups);
    [~, o] = sortrows([ia(:), is(:)]);
    idx = idx(o);
end

function SC = scenarioTable(S, sets, angOrder, sups, thr)
% One row per set x sun angle: detection / error at each supplementary level,
% easy/hard class, and the lowest supplementary level that recovers detection.
    SC = struct('label', {{}}, 'cls', {{}}, 'rec', {{}}, 'best', {{}}, ...
                'det', zeros(0, numel(sups)), 'px', zeros(0, numel(sups)), ...
                'py', zeros(0, numel(sups)), 'pz', zeros(0, numel(sups)), ...
                'ori', zeros(0, numel(sups)));
    iN = find(strcmp(sups, 'none'));
    for s = 1:numel(sets)
        for a = 1:numel(angOrder)
            if ~any(strcmp(S.set, sets{s}) & strcmp(S.sun, angOrder{a})), continue; end
            det = arrayfun(@(r) getMetric(S, sets{s}, angOrder{a}, sups{r}, 'detRate'), 1:numel(sups));
            px  = arrayfun(@(r) getMetric(S, sets{s}, angOrder{a}, sups{r}, 'posX_rmse'), 1:numel(sups));
            py  = arrayfun(@(r) getMetric(S, sets{s}, angOrder{a}, sups{r}, 'posY_rmse'), 1:numel(sups));
            pz  = arrayfun(@(r) getMetric(S, sets{s}, angOrder{a}, sups{r}, 'posZ_rmse'), 1:numel(sups));
            ori = arrayfun(@(r) getMetric(S, sets{s}, angOrder{a}, sups{r}, 'oriTot_rmse'), 1:numel(sups));
            dn = NaN;
            if ~isempty(iN), dn = det(iN); end
            if isnan(dn),     cls = 'unknown';
            elseif dn >= thr, cls = 'easy';
            else,             cls = 'hard';
            end
            rec = '-';
            if strcmp(cls, 'hard')
                ok = find(det >= thr & ~strcmp(sups, 'none'), 1);
                if isempty(ok), rec = 'not recovered'; else, rec = sups{ok}; end
            end
            best = '-';
            if any(~isnan(det)), [~, ib] = max(det); best = sups{ib}; end
            SC.label{end+1} = sprintf('%s, %s', sets{s}, angLabel(angOrder{a}));
            SC.cls{end+1} = cls;  SC.rec{end+1} = rec;  SC.best{end+1} = best;
            SC.det(end+1, :) = det;  SC.ori(end+1, :) = ori;
            SC.px(end+1, :) = px;  SC.py(end+1, :) = py;  SC.pz(end+1, :) = pz;
        end
    end
end

% ---------------------------------------------------------------------------
function fig = compareFigure(S, sun, sups, sets, setCol, thr)
% Baseline vs dark at one sun angle, across supplementary light levels.
% Position error is shown per camera axis (X lateral, Y vertical, Z depth).
    fig = [];
    m = strcmp(S.sun, sun);
    if ~any(m)
        fprintf('\n(No trials at sun angle "%s": comparison figure skipped.)\n', angLabel(sun));
        return;
    end
    sups = sups(ismember(sups, S.sup(m)));
    nS = numel(sups);
    if nS == 0, return; end
    g  = @(set, f) arrayfun(@(r) getMetric(S, set, sun, sups{r}, f), 1:nS)';
    gg = @(f) [g(sets{1}, f), g(sets{2}, f)];          % nS x 2 (baseline, dark)

    % ---- printed table ----
    fprintf('\n============ COMPARISON AT SUN %s: baseline vs dark, by supplementary light ============\n', ...
            angLabel(sun));
    fprintf('  %-9s %-5s | %6s | %-20s | %-20s | %-20s | %-20s\n', 'set', 'sup', 'det %', ...
            'X mean/std/rmse', 'Y mean/std/rmse', 'Z mean/std/rmse', 'ori mean/std/rmse');
    for s = 1:numel(sets)
        for r = 1:nS
            q = @(f) getMetric(S, sets{s}, sun, sups{r}, f);
            if isnan(q('detRate')) && isnan(q('posX_rmse')), continue; end
            fprintf(['  %-9s %-5s | %6.1f | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f | ' ...
                     '%6.1f %6.1f %6.1f | %6.2f %6.2f %6.2f\n'], ...
                    sets{s}, sups{r}, q('detRate'), ...
                    q('posX_mean'), q('posX_std'), q('posX_rmse'), ...
                    q('posY_mean'), q('posY_std'), q('posY_rmse'), ...
                    q('posZ_mean'), q('posZ_std'), q('posZ_rmse'), ...
                    q('oriTot_mean'), q('oriTot_std'), q('oriTot_rmse'));
        end
    end
    fprintf('  (X / Y / Z in mm, camera frame; ori in deg)\n');

    % ---- figure: 2 x 4 tiles ----
    % Row 1: detection rate, X / Y / Z RMSE.
    % Row 2: orientation RMSE, X / Y / Z mean error +/- std (bias and spread).
    fig = figure('Name', sprintf('Baseline vs dark at %s', angLabel(sun)), 'Color', 'w', ...
                 'Position', [60 40 1750 820]);
    tl = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    axNames = {'X', 'X lateral'; 'Y', 'Y vertical'; 'Z', 'Z depth'};

    ax = nexttile(tl);
    groupedBar(ax, sups, gg('detRate'), sets, setCol);
    ylim(ax, [0 110]); hline(ax, thr);
    ylabel(ax, 'Detection rate (%)'); title(ax, 'Detection rate');
    legend(ax, sets, 'Location', 'best');

    for a = 1:3
        ax = nexttile(tl);
        groupedBar(ax, sups, gg(['pos' axNames{a,1} '_rmse']), sets, setCol);
        ylabel(ax, 'mm'); title(ax, sprintf('%s position RMSE', axNames{a,2}));
    end

    ax = nexttile(tl);
    groupedBar(ax, sups, gg('oriTot_rmse'), sets, setCol);
    ylabel(ax, 'deg'); title(ax, 'Orientation RMSE (total)');

    for a = 1:3
        ax = nexttile(tl);
        M  = gg(['pos' axNames{a,1} '_mean']);
        SD = gg(['pos' axNames{a,1} '_std']);
        b = groupedBar(ax, sups, M, sets, setCol);
        hold(ax, 'on');
        for s = 1:numel(b)
            x = b(s).XEndPoints(1:nS);
            errorbar(ax, x, M(:, s)', SD(:, s)', 'k', 'LineStyle', 'none', ...
                     'HandleVisibility', 'off');
        end
        h0 = yline(ax, 0, 'k-');  h0.HandleVisibility = 'off';
        ylabel(ax, 'mm'); title(ax, sprintf('%s mean error \\pm std', axNames{a,2}));
    end

    for k = 1:numel(tl.Children)
        if isa(tl.Children(k), 'matlab.graphics.axis.Axes')
            xlabel(tl.Children(k), 'Supplementary light');
        end
    end
    title(tl, sprintf(['Baseline vs dark at sun %s, by supplementary light level ' ...
                       '(position error per camera axis)'], angLabel(sun)));
end

% ---------------------------------------------------------------------------
function b = groupedBar(ax, xlabels, Y, names, cols)
% Grouped bars that also work for a single x category.
    nx = size(Y, 1);
    if nx == 1
        b = bar(ax, [Y; nan(1, size(Y, 2))]);
    else
        b = bar(ax, Y);
    end
    for k = 1:numel(b)
        b(k).DisplayName = names{k};
        if nargin >= 5 && k <= size(cols, 1), b(k).FaceColor = cols(k, :); end
    end
    xticks(ax, 1:nx); xticklabels(ax, xlabels); xlim(ax, [0.5 nx + 0.5]);
    if nx > 3, xtickangle(ax, 30); end
    grid(ax, 'on');
end

function drawHeat(ax, M, xl, yl, cl, cmap, fmt)
    imagesc(ax, M, 'AlphaData', ~isnan(M));
    set(ax, 'Color', [0.88 0.88 0.88]);
    colormap(ax, cmap); caxis(ax, cl); colorbar(ax);
    xticks(ax, 1:numel(xl)); xticklabels(ax, xl);
    yticks(ax, 1:numel(yl)); yticklabels(ax, yl);
    xlabel(ax, 'Sun angle'); ylabel(ax, 'Supplementary light');
    for r = 1:size(M, 1)
        for c = 1:size(M, 2)
            if ~isnan(M(r, c))
                text(ax, c, r, sprintf(fmt, M(r, c)), 'HorizontalAlignment', 'center', ...
                     'FontWeight', 'bold');
            end
        end
    end
    axis(ax, 'tight');
end

function c = divMap(n)
% Red (worse) - white - blue (better)
    h = floor(n / 2);
    c = [[linspace(0.70, 1, h)', linspace(0.10, 1, h)', linspace(0.15, 1, h)'];
         [linspace(1, 0.13, h)', linspace(1, 0.40, h)', linspace(1, 0.70, h)']];
end

function hline(ax, y)
% Dotted threshold line that does not appear in legends.
    h = yline(ax, y, ':');
    h.HandleVisibility = 'off';
end

function noData(ax, msg)
    text(ax, 0.5, 0.5, msg, 'Units', 'normalized', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
end

% ---------------------------------------------------------------------------
function printDetection(S, set, angOrder, sups)
    idx = sortedIdx(S, set, angOrder, sups);
    if isempty(idx), fprintf('  (no trials)\n'); return; end
    fprintf('  %-40s %-8s %-5s %11s %8s\n', 'bag', 'sun', 'sup', 'detected', 'rate');
    for i = idx(:)'
        fprintf('  %-40s %-8s %-5s %5g / %-5g %6.1f %%\n', S.bag{i}, angLabel(S.sun{i}), ...
                S.sup{i}, S.nDetected(i), S.nFrames(i), S.detRate(i));
    end
    fprintf('  Mean detection rate: %.1f %% over %d trials\n', ...
            mean(S.detRate(idx), 'omitnan'), numel(idx));
end

function printStats(S, idx, header)
    fprintf('\n===== %s =====\n', header);
    if isempty(idx), fprintf('  (no trials)\n'); return; end
    fprintf('POSITION ERROR (mm, camera frame)\n');
    fprintf('  %-36s %-22s %5s | %-20s | %-20s | %-20s\n', 'bag', 'condition', 'n', ...
            'X lateral m/s/rmse', 'Y vertical m/s/rmse', 'Z depth m/s/rmse');
    for i = idx(:)'
        fprintf(['  %-36s %-22s %5d | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f | ' ...
                 '%6.1f %6.1f %6.1f\n'], ...
                S.bag{i}, condLabel(S, i), S.nUsed(i), ...
                S.posX_mean(i), S.posX_std(i), S.posX_rmse(i), ...
                S.posY_mean(i), S.posY_std(i), S.posY_rmse(i), ...
                S.posZ_mean(i), S.posZ_std(i), S.posZ_rmse(i));
    end
    fprintf('ORIENTATION ERROR (deg, marker frame)\n');
    fprintf('  %-36s %-22s %5s | %-20s | %-20s | %-20s | %-20s\n', 'bag', 'condition', 'n', ...
            'about X m/s/rmse', 'about Y m/s/rmse', 'about Z m/s/rmse', 'total mean/rmse/max');
    for i = idx(:)'
        fprintf(['  %-36s %-22s %5d | %6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f | ' ...
                 '%6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f\n'], ...
                S.bag{i}, condLabel(S, i), S.nUsed(i), ...
                S.oriX_mean(i), S.oriX_std(i), S.oriX_rmse(i), ...
                S.oriY_mean(i), S.oriY_std(i), S.oriY_rmse(i), ...
                S.oriZ_mean(i), S.oriZ_std(i), S.oriZ_rmse(i), ...
                S.oriTot_mean(i), S.oriTot_rmse(i), S.oriTot_max(i));
    end
    notes = idx(~cellfun(@isempty, S.note(idx)));
    for i = notes(:)'
        fprintf('  note: %s: %s\n', S.bag{i}, S.note{i});
    end
end

function s = condLabel(S, i)
    s = sprintf('%s | sup %s', angLabel(S.sun{i}), S.sup{i});
end

% ---------------------------------------------------------------------------
function P = readPoses(bag, topic)
% [t x y z qx qy qz qw], t from the header stamp, sorted, duplicates removed.
    P = cleanPoses(readPosesRaw(bag, topic));
end

function P = cleanPoses(P)
% Sort by time and keep one pose per timestamp.
    if isempty(P), P = zeros(0, 8); return; end
    [~, idx] = unique(P(:,1));
    P = P(idx, :);
end

function P = readPosesRaw(bag, topic)
% [t x y z qx qy qz qw] in message order (no sorting or de-duplication), so that
% rows stay aligned with other topics published alongside, e.g. /aruco/id.
    P = zeros(0, 8);
    try
        sel = select(bag, 'Topic', topic);
    catch
        return;
    end
    if sel.NumMessages == 0, return; end
    msgs = readMessages(sel);
    P = zeros(numel(msgs), 8);
    for k = 1:numel(msgs)
        m = msgs{k};
        P(k,:) = [double(m.header.stamp.sec) + 1e-9 * double(m.header.stamp.nanosec), ...
                  m.pose.position.x, m.pose.position.y, m.pose.position.z, ...
                  m.pose.orientation.x, m.pose.orientation.y, ...
                  m.pose.orientation.z, m.pose.orientation.w];
    end
end

function d = readInts(bag, topic)
    d = [];
    try
        sel = select(bag, 'Topic', topic);
    catch
        return;
    end
    if sel.NumMessages == 0, return; end
    msgs = readMessages(sel);
    d = cellfun(@(m) double(m.data), msgs(:));
end

function in = inWindows(t, W, margin)
% True where t falls inside any [start end] window (+/- margin).
    in = false(size(t));
    for w = 1:size(W, 1)
        in = in | (t >= W(w,1) - margin & t <= W(w,2) + margin);
    end
end

function W = findDropouts(P, minFrozen, jumpThr)
% [t_start t_end] windows where mocap froze (identical poses) or jumped.
    W = zeros(0, 2);
    if size(P, 1) < 2, return; end
    same = all(abs(diff(P(:,2:8), 1, 1)) < 1e-9, 2);
    k = 1; n = numel(same);
    while k <= n
        if same(k)
            j = k;
            while j <= n && same(j), j = j + 1; end
            if (j - k) >= minFrozen
                W(end+1, :) = [P(k,1), P(j,1)]; %#ok<AGROW>
            end
            k = j;
        else
            k = k + 1;
        end
    end
    jumps = find(sqrt(sum(diff(P(:,2:4), 1, 1).^2, 2)) > jumpThr);
    for j = jumps(:)'
        W(end+1, :) = [P(j,1), P(j+1,1)]; %#ok<AGROW>
    end
end

function out = findOutliers(E, valid, win, k, minDev)
% Frames whose error departs from the running median of their neighbours.
    E(~valid, :) = NaN;
    dev = sqrt(sum((E - movmedian(E, win, 1, 'omitnan')).^2, 2));
    dev(~valid) = NaN;
    locMed = movmedian(dev, 31, 'omitnan');
    locMAD = movmedian(abs(dev - locMed), 31, 'omitnan');
    thr = max(minDev, k * 1.4826 * locMAD);
    out = valid & dev > thr;
end

function T = interpPose(P, tq)
% Pose at time tq: linear position, normalised-lerp quaternion.
    tq = min(max(tq, P(1,1)), P(end,1));
    if size(P, 1) == 1, T = poseToT(P(1,2:8)); return; end
    i = max(1, min(size(P,1) - 1, find(P(:,1) <= tq, 1, 'last')));
    s = (tq - P(i,1)) / (P(i+1,1) - P(i,1));
    q1 = P(i,5:8); q2 = P(i+1,5:8);
    if dot(q1, q2) < 0, q2 = -q2; end
    T = poseToT([(1-s) * P(i,2:4) + s * P(i+1,2:4), (1-s) * q1 + s * q2]);
end

function T = medianPose(P)
% Median pose of a (nearly) static stream, quaternions aligned first.
    q = P(:,5:8);
    flip = q * q(1,:)' < 0;
    q(flip, :) = -q(flip, :);
    T = poseToT([median(P(:,2:4), 1), median(q, 1)]);
end

function T = meanPose(Ts)
% Median position and chordal-mean rotation of a stack of 4x4 poses.
    pos = reshape(Ts(1:3,4,:), 3, [])';
    M = sum(Ts(1:3,1:3,:), 3);
    [U, ~, V] = svd(M);
    T = eye(4);
    T(1:3,1:3) = U * diag([1 1 det(U * V')]) * V';
    T(1:3,4) = median(pos, 1)';
end

function T = poseToT(v)
% v = [x y z qx qy qz qw]
    q = v(4:7) / norm(v(4:7)); x = q(1); y = q(2); z = q(3); w = q(4);
    T = eye(4);
    T(1:3,1:3) = [1-2*(y^2+z^2), 2*(x*y-z*w),   2*(x*z+y*w);
                  2*(x*y+z*w),   1-2*(x^2+z^2), 2*(y*z-x*w);
                  2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x^2+y^2)];
    T(1:3,4) = v(1:3)';
end

function rv = rotmToRotvec(Rm)
% Rotation matrix -> rotation vector (axis * angle, rad)
    c  = max(-1, min(1, (trace(Rm) - 1) / 2));
    th = acos(c);
    if th < 1e-9
        rv = [0; 0; 0];
    elseif pi - th < 1e-6
        [V, Dg] = eig(Rm); [~, i] = min(abs(diag(Dg) - 1));
        rv = th * real(V(:, i));
    else
        rv = th / (2 * sin(th)) * [Rm(3,2) - Rm(2,3); Rm(1,3) - Rm(3,1); Rm(2,1) - Rm(1,2)];
    end
end

function s = errStats(E)
% Rows: mean, std, RMSE. Columns: axes. NaN when there is no data.
    if isempty(E), s = nan(3, max(size(E, 2), 1)); return; end
    s = [mean(E, 1); std(E, 0, 1); sqrt(mean(E.^2, 1))];
end

function v = safeMax(x)
    if isempty(x), v = NaN; else, v = max(x); end
end
% ===========================================================================
%  OBJECTIVE ANALYSES: helper functions
% ===========================================================================
function NF = noiseFloor(S, MINN, K)
% Noise floor from the repeated Ambient reference trials (no sun, no
% supplementary light). thr = max(K x repeat std, a fixed minimum).
    rep = strcmp(S.set, 'baseline') & strcmp(S.sun, 'none') & strcmp(S.sup, 'none');
    NF.nRepeats = sum(rep);
    NF.K = K;
    NF.metrics = {'detRate', 'pos3D_rmse', 'pos3D_std', 'pos3D_mean', 'posX_rmse', ...
                  'posY_rmse', 'posZ_rmse', 'oriTot_rmse', 'oriTot_std', 'oriTot_max', 'flipRate'};
    for k = 1:numel(NF.metrics)
        f = NF.metrics{k};
        v = S.(f)(rep);
        v = v(~isnan(v));
        if numel(v) >= 2, sd = std(v); else, sd = NaN; end
        if strncmp(f, 'pos', 3),       mn = MINN.pos_mm;
        elseif strncmp(f, 'ori', 3),   mn = MINN.ori_deg;
        elseif strcmp(f, 'flipRate'),  mn = MINN.flip_pct;
        else,                          mn = MINN.det_pct;
        end
        if isempty(v), m = NaN; else, m = mean(v); end
        t = mn;
        if ~isnan(sd), t = max(K * sd, mn); end
        NF.(f) = struct('mean', m, 'sd', sd, 'thr', t);
    end
end

function t = nfThr(NF, f)
    if isfield(NF, f), t = NF.(f).thr; else, t = Inf; end
end

function [names, rows] = noiseFloorRows(NF)
    names = {'metric', 'reference_mean', 'repeat_sd', 'threshold'};
    rows = cell(numel(NF.metrics), 4);
    for k = 1:numel(NF.metrics)
        f = NF.metrics{k};
        rows(k, :) = {f, NF.(f).mean, NF.(f).sd, NF.(f).thr};
    end
end

function rows = repeatSpread(S)
% Spread of every condition recorded more than once.
    rows = cell(0, 7);
    key = strcat(S.set(:), '|', S.sun(:), '|', S.sup(:));
    done = false(numel(key), 1);
    for q = 1:numel(key)
        if done(q), continue; end
        m = find(strcmp(key, key{q}));
        done(m) = true;
        if numel(m) < 2, continue; end
        sdv = @(f) std(S.(f)(m(~isnan(S.(f)(m)))));
        i = m(1);
        rows(end+1, :) = {S.set{i}, S.sun{i}, S.sup{i}, numel(m), sdv('detRate'), ...
                          sdv('pos3D_rmse'), sdv('oriTot_rmse')}; %#ok<AGROW>
    end
end

function S = addDeltas(S, metrics)
% dRef_<m>: change vs the ideal reference (Ambient, no sun, no supplementary light)
% dSup_<m>: change vs no supplementary light at the same set and sun angle
    n = numel(S.bag);
    for k = 1:numel(metrics)
        f = metrics{k};
        dRef = nan(n, 1);  dSup = nan(n, 1);
        ref = getMetric(S, 'baseline', 'none', 'none', f);
        for i = 1:n
            dRef(i) = S.(f)(i) - ref;
            dSup(i) = S.(f)(i) - getMetric(S, S.set{i}, S.sun{i}, 'none', f);
        end
        S.(['dRef_' f]) = dRef;
        S.(['dSup_' f]) = dSup;
    end
end

function mode = classifyTrials(S, NF, thr)
% Failure mode of each trial relative to the ideal reference (literature
% failure types: no detection / no identification / inaccurate pose).
    n = numel(S.bag);
    mode = cell(n, 1);
    for i = 1:n
        d = S.detRate(i);
        if isnan(d)
            m = 'no data';
        elseif d == 0
            m = 'no detection';
        else
            if d < thr, m = 'intermittent detection'; else, m = 'detected'; end
            tags = {};
            if S.dRef_pos3D_rmse(i) > nfThr(NF, 'pos3D_rmse') || ...
               S.dRef_oriTot_rmse(i) > nfThr(NF, 'oriTot_rmse')
                tags{end+1} = 'less accurate'; %#ok<AGROW>
            end
            if S.dRef_pos3D_std(i) > nfThr(NF, 'pos3D_std') || ...
               S.dRef_oriTot_std(i) > nfThr(NF, 'oriTot_std') || ...
               S.dRef_flipRate(i) > nfThr(NF, 'flipRate')
                tags{end+1} = 'less stable'; %#ok<AGROW>
            end
            if ~isempty(tags)
                m = [m ', ' strjoin(tags, ', ')]; %#ok<AGROW>
            elseif d >= thr
                m = 'no measurable degradation';
            end
        end
        if ~isnan(S.nWrongID(i)) && S.nWrongID(i) > 0
            m = sprintf('%s; %d wrong-ID detections', m, S.nWrongID(i));
        end
        mode{i} = m;
    end
end

function L = supLux(S, LUX, sups)
% Measured supplementary illuminance for each trial (NaN if not measured).
    n = numel(S.bag);
    L = nan(n, 1);
    for i = 1:n
        k = find(strcmp(sups, S.sup{i}), 1);
        if isfield(LUX, S.set{i}) && ~isempty(k) && k <= numel(LUX.(S.set{i}))
            L(i) = LUX.(S.set{i})(k);
        end
    end
end

function v = changeVerdict(S, i, NF, pre)
% Compare position RMSE, position std, orientation RMSE and orientation jumps
% (columns <pre><metric>) with the noise floor. With pre = 'dSup_', if the
% no-supplementary trial had no poses to compare with (0 % detection), the
% comparison falls back to the ideal reference (dRef_) and says so.
    chk = {'pos3D_rmse', 'position RMSE'; 'pos3D_std', 'position std';
           'oriTot_rmse', 'orientation RMSE'; 'flipRate', 'orientation jumps'};
    suffix = '';
    if strcmp(pre, 'dSup_') && all(cellfun(@(f) isnan(S.(['dSup_' f])(i)), chk(:,1)))
        pre = 'dRef_';
        suffix = ' (vs ideal reference: no poses without supplementary light)';
    end
    worse = {};  better = {};  have = false;
    for k = 1:size(chk, 1)
        d = S.([pre chk{k,1}])(i);
        if isnan(d), continue; end
        have = true;
        t = nfThr(NF, chk{k,1});
        if d > t
            worse{end+1} = chk{k,2}; %#ok<AGROW>
        elseif d < -t
            better{end+1} = chk{k,2}; %#ok<AGROW>
        end
    end
    if ~have
        v = 'n/a';
    elseif isempty(worse) && isempty(better)
        v = 'within noise';
    else
        parts = {};
        if ~isempty(worse),  parts{end+1} = ['worse: ' strjoin(worse, ', ')]; end
        if ~isempty(better), parts{end+1} = ['better: ' strjoin(better, ', ')]; end
        v = strjoin(parts, '; ');
    end
    if ~strcmp(v, 'n/a'), v = [v suffix]; end
end

function [names, rows] = objective1Rows(S, sets, angOrder)
    names = {'set', 'sun', 'det_pct', 'd_det_pct', 'pos3D_rmse_mm', 'd_pos3D_rmse_mm', ...
             'pos3D_bias_mm', 'pos3D_std_mm', 'd_pos3D_std_mm', 'ori_rmse_deg', ...
             'd_ori_rmse_deg', 'ori_max_deg', 'jumps_pct', 'wrong_id', 'n_used', 'failure_mode'};
    rows = cell(0, numel(names));
    for s = 1:numel(sets)
        idx = sortedIdx(S, sets{s}, angOrder, {'none'});
        idx = idx(strcmp(S.sup(idx), 'none'));
        for i = idx(:)'
            rows(end+1, :) = {S.set{i}, S.sun{i}, S.detRate(i), S.dRef_detRate(i), ...
                S.pos3D_rmse(i), S.dRef_pos3D_rmse(i), S.pos3D_mean(i), S.pos3D_std(i), ...
                S.dRef_pos3D_std(i), S.oriTot_rmse(i), S.dRef_oriTot_rmse(i), ...
                S.oriTot_max(i), S.flipRate(i), S.nWrongID(i), S.nUsed(i), S.failMode{i}}; %#ok<AGROW>
        end
    end
end

function [names, rows, reqNames, reqRows, interferenceOK] = objective2Rows(S, sets, sups, NF, thr, tolPct, LUX)
    names = {'set', 'sup', 'lux', 'det_pct', 'd_det_pct', 'pos3D_rmse_mm', 'd_pos3D_rmse_mm', ...
             'd_pos3D_std_mm', 'd_ori_rmse_deg', 'd_ori_max_deg', 'd_jumps_pct', 'change'};
    rows = cell(0, numel(names));
    minDet = Inf;  nWorse = 0;  nRows = 0;
    for s = 1:numel(sets)
        for k = 1:numel(sups)
            if strcmp(sups{k}, 'none'), continue; end
            m = find(strcmp(S.set, sets{s}) & strcmp(S.sun, 'none') & strcmp(S.sup, sups{k}));
            for i = m(:)'
                v = changeVerdict(S, i, NF, 'dSup_');
                rows(end+1, :) = {S.set{i}, S.sup{i}, S.supLux(i), S.detRate(i), ...
                    S.dSup_detRate(i), S.pos3D_rmse(i), S.dSup_pos3D_rmse(i), ...
                    S.dSup_pos3D_std(i), S.dSup_oriTot_rmse(i), S.dSup_oriTot_max(i), ...
                    S.dSup_flipRate(i), v}; %#ok<AGROW>
                if ~isnan(S.detRate(i)), minDet = min(minDet, S.detRate(i)); end
                nRows = nRows + 1;
                if strncmp(v, 'worse', 5), nWorse = nWorse + 1; end
            end
        end
    end

    % OptiTrack interference: dropout percentage in trials that use their own
    % OptiTrack data (baseline set), lamp off vs lamp on, over all sun angles.
    hasMocap = ~isnan(S.mocapDropPct);
    off = hasMocap & strcmp(S.sup, 'none');
    on  = hasMocap & ~strcmp(S.sup, 'none');
    dropOff = mean(S.mocapDropPct(off));
    dropOn  = mean(S.mocapDropPct(on));

    reqNames = {'requirement', 'result', 'status'};
    reqRows = cell(0, 3);
    if isinf(minDet)
        reqRows(end+1, :) = {sprintf('Lamp alone keeps detection >= %g %%', thr), 'no lamp-only trials', 'not tested'};
    else
        reqRows(end+1, :) = {sprintf('Lamp alone keeps detection >= %g %%', thr), ...
                             sprintf('lowest %.0f %%', minDet), passFail(minDet >= thr)};
    end
    if nRows == 0
        reqRows(end+1, :) = {'Lamp alone does not reduce accuracy or stability', 'no lamp-only trials', 'not tested'};
    else
        reqRows(end+1, :) = {'Lamp alone does not reduce accuracy or stability', ...
                             sprintf('%d of %d lamp-only trials worse than noise floor', nWorse, nRows), ...
                             passFail(nWorse == 0)};
    end
    if any(off) && any(on)
        interferenceOK = (dropOn - dropOff) <= tolPct;
        reqRows(end+1, :) = {sprintf('No OptiTrack interference (<= %g pp extra dropout)', tolPct), ...
                             sprintf('dropout %.1f %% lamp off, %.1f %% lamp on', dropOff, dropOn), ...
                             passFail(interferenceOK)};
    else
        interferenceOK = NaN;
        reqRows(end+1, :) = {sprintf('No OptiTrack interference (<= %g pp extra dropout)', tolPct), ...
                             'no OptiTrack data with lamp on and off', 'not tested'};
    end
    for s = 1:numel(sets)
        if ~isfield(LUX, sets{s}), continue; end
        lx = LUX.(sets{s});
        if any(isnan(lx))
            reqRows(end+1, :) = {sprintf('Distinct illuminance levels (%s)', sets{s}), ...
                                 'not all levels measured', 'not measured'}; %#ok<AGROW>
        else
            reqRows(end+1, :) = {sprintf('Distinct illuminance levels (%s)', sets{s}), ...
                                 strjoin(arrayfun(@(x) sprintf('%g', x), lx, 'UniformOutput', false), ' / '), ...
                                 passFail(all(diff(lx) > 0))}; %#ok<AGROW>
        end
    end
end

function [names, rows] = objective3Rows(S, sets, angOrder, sups, NF)
    names = {'set', 'sun', 'sup', 'lux', 'det_pct', 'd_det_pct', 'pos3D_rmse_mm', ...
             'd_pos3D_rmse_mm', 'pos3D_std_mm', 'd_pos3D_std_mm', 'ori_rmse_deg', ...
             'd_ori_rmse_deg', 'ori_max_deg', 'jumps_pct', 'd_jumps_pct', ...
             'dref_pos3D_rmse_mm', 'dref_pos3D_std_mm', 'change', 'proc_ms'};
    rows = cell(0, numel(names));
    for s = 1:numel(sets)
        idx = sortedIdx(S, sets{s}, angOrder, sups);
        idx = idx(~strcmp(S.sun(idx), 'none'));
        for i = idx(:)'
            if strcmp(S.sup{i}, 'none')
                v = 'no supplementary light';
            else
                v = changeVerdict(S, i, NF, 'dSup_');
            end
            rows(end+1, :) = {S.set{i}, S.sun{i}, S.sup{i}, S.supLux(i), S.detRate(i), ...
                S.dSup_detRate(i), S.pos3D_rmse(i), S.dSup_pos3D_rmse(i), S.pos3D_std(i), ...
                S.dSup_pos3D_std(i), S.oriTot_rmse(i), S.dSup_oriTot_rmse(i), ...
                S.oriTot_max(i), S.flipRate(i), S.dSup_flipRate(i), S.dRef_pos3D_rmse(i), ...
                S.dRef_pos3D_std(i), v, S.procMean_ms(i)}; %#ok<AGROW>
        end
    end
end

function [names, rows, verdict] = objective5Rows(S, sets, angOrder, sups, NF, thr, ...
                                                 tgtMM, tgtDeg, fps, LUX, sunLux, interferenceOK)
% For each set and sun angle, take the lowest supplementary level that reaches the
% availability threshold (or the best level if none does) and check it against
% the ground-truth criteria.
    names = {'set', 'sun', 'level', 'det_pct', 'availability', 'pos3D_rmse_mm', ...
             'ori_rmse_deg', 'accuracy', 'd_pos3D_std_mm', 'd_jumps_pct', 'stability', ...
             'proc_ms', 'real_time', 'added_lux', 'sup_to_sun_ratio', 'optitrack'};
    rows = cell(0, numel(names));
    verdict = {};
    if isnan(interferenceOK), optTxt = 'not tested'; else, optTxt = passFail(interferenceOK); end
    for s = 1:numel(sets)
        nCond = 0;  nAvail = 0;  nAcc = 0;  nStab = 0;  nAccTested = 0;
        for a = 1:numel(angOrder)
            sun = angOrder{a};
            det = arrayfun(@(k) getMetric(S, sets{s}, sun, sups{k}, 'detRate'), 1:numel(sups));
            if all(isnan(det)), continue; end
            k = find(det >= thr, 1);
            if isempty(k), [~, k] = max(det); end
            lvl = sups{k};
            g = @(f) getMetric(S, sets{s}, sun, lvl, f);
            nCond = nCond + 1;

            avail = det(k) >= thr;
            nAvail = nAvail + avail;

            p3 = g('pos3D_rmse');  oR = g('oriTot_rmse');
            if isnan(tgtMM) && isnan(tgtDeg)
                acc = 'target not set';
            elseif isnan(p3)
                acc = 'no pose data';
            else
                ok = true;
                if ~isnan(tgtMM),  ok = ok && p3 <= tgtMM; end
                if ~isnan(tgtDeg), ok = ok && oR <= tgtDeg; end
                acc = passFail(ok);
                nAccTested = nAccTested + 1;
                nAcc = nAcc + ok;
            end

            dStd = g('dRef_pos3D_std');  dJmp = g('dRef_flipRate');
            if isnan(dStd)
                stab = 'n/a';
            else
                okS = dStd <= nfThr(NF, 'pos3D_std') && ...
                      (isnan(dJmp) || dJmp <= nfThr(NF, 'flipRate'));
                stab = passFail(okS);
                nStab = nStab + okS;
            end

            pm = g('procMean_ms');
            if isnan(pm), rt = 'not measured'; else, rt = passFail(pm <= 1000 / fps); end

            added = NaN;  ratio = NaN;
            if isfield(LUX, sets{s}) && k <= numel(LUX.(sets{s}))
                added = LUX.(sets{s})(k) - LUX.(sets{s})(1);
                if ~strcmp(sun, 'none'), ratio = added / sunLux; end
            end

            rows(end+1, :) = {sets{s}, sun, lvl, det(k), passFail(avail), p3, oR, acc, ...
                              dStd, dJmp, stab, pm, rt, added, ratio, optTxt}; %#ok<AGROW>
        end
        if nCond == 0, continue; end
        if nAccTested == 0
            accTxt = 'accuracy target not set (set ACCURACY_TARGET_MM / _DEG)';
        else
            accTxt = sprintf('accuracy met at %d of %d', nAcc, nAccTested);
        end
        verdict{end+1} = sprintf(['%s: availability met at %d of %d conditions; ' ...
                                  'stability no worse than the reference at %d of %d; %s'], ...
                                 upper(sets{s}), nAvail, nCond, nStab, nCond, accTxt); %#ok<AGROW>
    end
end

function s = passFail(ok)
    if ok, s = 'pass'; else, s = 'fail'; end
end

function printRows(header, names, rows)
% Print a cell array of rows as an aligned text table.
    fprintf('\n===== %s =====\n', header);
    if isempty(rows), fprintf('  (no rows)\n'); return; end
    txt = cell(size(rows));
    for r = 1:size(rows, 1)
        for c = 1:size(rows, 2)
            txt{r, c} = fmtCell(rows{r, c});
        end
    end
    w = max([cellfun(@numel, names(:)'); cellfun(@numel, txt)], [], 1);
    fprintf('  ');
    for c = 1:numel(names), fprintf('%-*s  ', w(c), names{c}); end
    fprintf('\n');
    for r = 1:size(rows, 1)
        fprintf('  ');
        for c = 1:numel(names), fprintf('%-*s  ', w(c), txt{r, c}); end
        fprintf('\n');
    end
end

function s = fmtCell(v)
    if ischar(v)
        s = v;
    elseif islogical(v) && isscalar(v)
        if v, s = 'yes'; else, s = 'no'; end
    elseif isnumeric(v) && isscalar(v)
        if isnan(v)
            s = '-';
        elseif v == round(v) && abs(v) < 1e6
            s = sprintf('%d', v);
        else
            s = sprintf('%.2f', v);
        end
    else
        s = '?';
    end
end

function ok = writeRows(file, names, rows)
% Save rows as a CSV (returns true so it can be used in a short-circuit).
    ok = true;
    if isempty(rows), return; end
    writetable(cell2table(rows, 'VariableNames', names), file);
end
function [codes, words] = failureGridLamp(S, rows, angOrder, NF, thr)
% Codes: 1 no detection, 2 intermittent, 3 degraded, 4 no change, 5 reference.
% Each cell is compared with the same environment and lamp level with no sun.
    accM = {'pos3D_rmse', 'oriTot_rmse'};
    stbM = {'pos3D_std', 'oriTot_std', 'flipRate'};
    codes = nan(size(rows, 1), numel(angOrder));
    words = repmat({''}, size(rows, 1), numel(angOrder));
    for r = 1:size(rows, 1)
        set = rows{r,1};  sup = rows{r,2};
        for a = 1:numel(angOrder)
            i = find(strcmp(S.set, set) & strcmp(S.sun, angOrder{a}) & strcmp(S.sup, sup), 1);
            if isempty(i), continue; end
            d = S.detRate(i);
            if isnan(d), continue; end
            if d == 0,       codes(r,a) = 1; words{r,a} = 'no detection'; continue; end
            if d < thr,      codes(r,a) = 2; words{r,a} = sprintf('%.0f%% detected', d); continue; end
            if strcmp(angOrder{a}, 'none')
                codes(r,a) = 5; words{r,a} = 'reference'; continue;
            end
            dv = @(m) S.(m)(i) - getMetric(S, set, 'none', sup, m);
            acc = any(cellfun(@(m) dv(m) > nfThr(NF, m), accM));
            stb = any(cellfun(@(m) dv(m) > nfThr(NF, m), stbM));
            if all(cellfun(@(m) isnan(dv(m)), [accM stbM]))
                codes(r,a) = 4; words{r,a} = 'detected (no reference)';
            elseif acc || stb
                tags = {};
                if acc, tags{end+1} = 'less accurate'; end %#ok<AGROW>
                if stb, tags{end+1} = 'less stable';   end %#ok<AGROW>
                codes(r,a) = 3; words{r,a} = strjoin(tags, ', ');
            else
                codes(r,a) = 4; words{r,a} = 'no change';
            end
        end
    end
end