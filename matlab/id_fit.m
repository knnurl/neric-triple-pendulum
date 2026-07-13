function fit = id_fit(varargin)
%ID_FIT  Staged grey-box fit of the cart + single-link model from ID runs.
%
%  Requires the System Identification Toolbox (idnlgrey / nlgreyest).
%
%  Usage:
%    fit = id_fit('pulley_circ', 0.072, ...
%                 'free',  {'id_free_20260704_101500.mat', ...}, ...
%                 'chirp', 'id_chirp_20260704_102200.mat', ...
%                 'val',   'id_prbs_20260704_102900.mat');
%
%  Name-value arguments:
%    pulley_circ  REQUIRED. Cart travel per motor rev [m/rev]. Measure it:
%                 jog the cart, read Δrev in the GUI, tape-measure Δx.
%    free         Cell array of free-swing captures (id_logger('free',...)).
%                 Hand-start a swing, cart held still. Pendulum params.
%    chirp        One chirp capture (id_logger('chirp',...)). Full joint fit.
%    val          Held-out capture for validation (NEVER one used to fit).
%    guess        Optional struct of initial guesses to override defaults:
%                 .Mt [kg] .ml [kg m] .bc [N s/m] .Fc [N]
%
%  Staged fitting (far more robust than one joint fit — Eltohamy & Kuo
%  identified friction on a reduced rig first for the same reason):
%    Stage 1  free swings -> pendulum-only fit of k1 = g*ml/Jt (stiffness)
%             and k2 = bp/Jt (damping). Averaged across files.
%    Stage 2  chirp -> full 6-parameter fit {Mt, ml, Jt, bc, Fc, bp},
%             initialized from stage 1 (Jt, bp seeded via the fixed ratios).
%    Stage 3  validation overlay + NRMSE fit % on the held-out run.
%
%  Output `fit` (also saved to id_fit_result.mat):
%    .params   fitted {Mt, ml, Jt, bc, Fc, bp, c} — feed to lqr_design.m
%    .stage1   per-file [k1 k2] and their mean
%    .model    the fitted idnlgrey object
%    .fitpct   stage-2 self-fit and stage-3 validation fit [%% NRMSE]

%% ---- Args ---------------------------------------------------------------
p = inputParser;
p.addParameter('pulley_circ', [], @(v) isnumeric(v) && v > 0);
p.addParameter('free',  {}, @(v) iscell(v) || ischar(v) || isstring(v));
p.addParameter('chirp', '', @(v) ischar(v) || isstring(v));
p.addParameter('val',   '', @(v) ischar(v) || isstring(v));
p.addParameter('guess', struct(), @isstruct);
p.parse(varargin{:});
a = p.Results;

if isempty(a.pulley_circ)
    error('id_fit:pulley', 'pulley_circ [m/rev] is required — measure it first.');
end
c = a.pulley_circ;
if ~iscell(a.free), a.free = cellstr(a.free); end

% Initial-guess defaults (override via 'guess')
G = struct('Mt', 1.5, 'ml', 0.05, 'bc', 5.0, 'Fc', 0.5);
fn = fieldnames(a.guess);
for k = 1:numel(fn), G.(fn{k}) = a.guess.(fn{k}); end

g  = 9.80665;
Ts = 1e-3;                      % idlog sample time (1 kHz)

%% ---- Stage 1: pendulum-only fit on free swings --------------------------
assert(~isempty(a.free), 'id_fit:nofree', 'At least one free-swing file is required.');

k12 = zeros(numel(a.free), 2);
for i = 1:numel(a.free)
    d  = load_run(a.free{i}, c, Ts);

    % Seed k1 from the dominant swing frequency (hanging: w^2 = g*ml/Jt).
    k1_guess = est_freq_seed(d.theta, Ts);

    z1 = iddata(d.theta, zeros(size(d.theta)), Ts, 'Name', a.free{i});
    m1 = idnlgrey(@pend_only_ode, [1 1 2], {k1_guess; 0.5}, ...
                  [d.theta(1); d.theta_dot(1)], 0, 'Name', 'pendulum-only');
    m1.Parameters(1).Name = 'k1 = g*ml/Jt';  m1.Parameters(1).Minimum = 0.1;
    m1.Parameters(2).Name = 'k2 = bp/Jt';    m1.Parameters(2).Minimum = 0;
    m1 = nlgreyest(z1, m1, nlgreyestOptions('Display', 'on'));

    k12(i,:) = [m1.Parameters(1).Value, m1.Parameters(2).Value];
    fprintf('  [%s]  k1 = %.3f 1/s^2   k2 = %.4f 1/s\n', a.free{i}, k12(i,1), k12(i,2));
end
k1 = mean(k12(:,1));  k2 = mean(k12(:,2));
fprintf('Stage 1: k1 = %.3f (std %.3f)   k2 = %.4f (std %.4f)\n', ...
        k1, std(k12(:,1)), k2, std(k12(:,2)));

%% ---- Stage 2: full joint fit on the chirp -------------------------------
assert(~isempty(a.chirp), 'id_fit:nochirp', 'A chirp file is required for stage 2.');
d  = load_run(a.chirp, c, Ts);
z2 = iddata([d.x_m, d.theta], d.u, Ts, 'Name', a.chirp);
z2.OutputName = {'x [m]', 'theta [rad]'};
z2.InputName  = {'torque [Nm]'};

% Seed Jt/bp from stage 1 through the guessed ml.
Jt0 = g * G.ml / k1;
bp0 = k2 * Jt0;

par0 = {G.Mt; G.ml; Jt0; G.bc; G.Fc; bp0; c};
m2 = idnlgrey(@cartpend_ode, [2 1 4], par0, ...
              [d.x_m(1); d.x_dot_m(1); d.theta(1); d.theta_dot(1)], 0, ...
              'Name', 'cart+link');
names = {'Mt','ml','Jt','bc','Fc','bp','c'};
mins  = [0.05, 1e-3, 1e-5, 0, 0, 0, c];
for k = 1:7, m2.Parameters(k).Name = names{k}; m2.Parameters(k).Minimum = mins(k); end
m2.Parameters(7).Fixed = true;                      % c is measured, not fitted

opt = nlgreyestOptions('Display', 'on', 'SearchMethod', 'auto');
opt.SearchOptions.MaxIterations = 60;
m2 = nlgreyest(z2, m2, opt);

P = struct();
for k = 1:7, P.(names{k}) = m2.Parameters(k).Value; end

[~, fit2] = compare(z2, m2);
fprintf('\nStage 2 self-fit: x %.1f %%   theta %.1f %%\n', fit2(1), fit2(2));

%% ---- Stage 3: validation on the held-out run ----------------------------
fit3 = [NaN NaN];
if ~isempty(a.val)
    dv = load_run(a.val, c, Ts);
    zv = iddata([dv.x_m, dv.theta], dv.u, Ts, 'Name', a.val);
    mv = m2;  mv.InitialStates = init_states(mv, ...
              [dv.x_m(1); dv.x_dot_m(1); dv.theta(1); dv.theta_dot(1)]);
    figure('Name', 'id\_fit validation', 'Color', 'w');
    compare(zv, mv);
    [~, fit3] = compare(zv, mv);
    fprintf('Stage 3 VALIDATION fit: x %.1f %%   theta %.1f %%\n', fit3(1), fit3(2));
    if fit3(2) < 80
        warning('id_fit:lowfit', ...
            'theta validation fit below 80%% — do not design gains on this yet. Suspects: friction model, theta sign, pulley_circ, encoder noise.');
    end
end

%% ---- Report + save ------------------------------------------------------
fprintf('\n===== Fitted parameters (SI, lumped) =====\n');
fprintf('  Mt = %8.4f kg      (M + m)\n',          P.Mt);
fprintf('  ml = %8.5f kg m    (m*l)\n',            P.ml);
fprintf('  Jt = %8.6f kg m^2  (J + m*l^2)\n',      P.Jt);
fprintf('  bc = %8.4f N s/m\n',                    P.bc);
fprintf('  Fc = %8.4f N       (Coulomb)\n',        P.Fc);
fprintf('  bp = %8.6f N m s\n',                    P.bp);
fprintf('  c  = %8.5f m/rev   (fixed)\n',          P.c);
fprintf('Sanity: hanging period = %.2f s (measured tape/stopwatch should agree)\n', ...
        2*pi / sqrt(g*P.ml/P.Jt));

fit = struct('params', P, 'stage1', struct('k12', k12, 'k1', k1, 'k2', k2), ...
             'model', m2, 'fitpct', struct('stage2', fit2, 'val', fit3), ...
             'when', datetime('now'));
save(fullfile(fileparts(mfilename('fullpath')), 'id_fit_result.mat'), '-struct', 'fit');
fprintf('\nSaved -> id_fit_result.mat.  Next: K = lqr_design();\n');

end

%% ======================================================================
%  Local helpers
%  ======================================================================

function d = load_run(fname, c, Ts)
    % Load an id_logger capture, unwrap theta, convert cart to SI, and
    % re-grid to uniform Ts if the ring dropped samples.
    r = load(fname);
    t = r.t(:);
    if any(abs(diff(t) - Ts) > Ts/4)
        warning('id_fit:gaps', '%s has sample gaps — re-gridding to %.0f Hz.', ...
                fname, 1/Ts);
        tu = (t(1):Ts:t(end))';
        f  = @(v) interp1(t, v(:), tu, 'linear');
        r.theta = f(r.theta); r.theta_dot = f(r.theta_dot);
        r.x = f(r.x); r.x_dot = f(r.x_dot); r.u = f(r.u);
    end
    d.theta     = unwrap(r.theta(:));    % hanging swings straddle the ±pi wrap
    d.theta_dot = r.theta_dot(:);
    d.x_m       = r.x(:)     * c;        % rev -> m
    d.x_dot_m   = r.x_dot(:) * c;
    d.u         = r.u(:);
end

function k1 = est_freq_seed(theta, Ts)
    % Seed the pendulum stiffness from the dominant oscillation frequency.
    th = detrend(theta);
    n  = 2^nextpow2(numel(th));
    Y  = abs(fft(th, n));
    fr = (0:n-1)' / (n*Ts);
    band = fr > 0.2 & fr < 10;           % plausible swing band
    [~, i] = max(Y(band));
    fb = fr(band);
    k1 = (2*pi*fb(i))^2;
    if ~isfinite(k1) || k1 <= 0, k1 = 30; end
end

function is = init_states(model, x0)
    is = model.InitialStates;
    for k = 1:numel(is), is(k).Value = x0(k); end
end

function [dx, y] = pend_only_ode(t, x, u, k1, k2, varargin) %#ok<INUSL>
    % Pendulum-only submodel for free swings (cart held still):
    %   thdd = k1*sin(th) - k2*thd,  th from upright (hanging ~ ±pi).
    dx = [x(2); k1*sin(x(1)) - k2*x(2)];
    y  = x(1);
end
