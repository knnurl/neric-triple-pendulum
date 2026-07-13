function out = swingup_design(varargin)
%SWINGUP_DESIGN  Tune + verify the energy swing-up on the fitted model.
%
%  Simulates EXACTLY the firmware law (swingup_ctrl.c) on the identified
%  plant (cartpend_ode.m), so gains proven here paste straight into
%  swingup_ctrl.h. Optionally chains the balance LQR after capture to
%  verify the handoff basin.
%
%  Usage (after id_fit; K from lqr_design for the handoff check):
%    out = swingup_design();                              % defaults
%    out = swingup_design('KE', 1.5, 'KX', 0.05, 'KXD', 0.02);
%    out = swingup_design('KE_sweep', [0.5 1 2 4 8]);     % pick a pump gain
%    out = swingup_design('K_si', K_si);                  % + LQR catch check
%
%  Name-values:
%    fitfile   id_fit_result.mat (default) | 'params' struct to bypass
%    KE        energy pump gain [Nm/(J rad/s)]        (SWING_KE)
%    KX, KXD   cart PD, FIRMWARE units [Nm/rev, Nm/(rev/s)] (SWING_KX/KXD)
%    u_max     torque clamp [Nm]                      (SWING_TORQUE_MAX_NM)
%    x_max     cart excursion budget from centre [m]
%    catch_th, catch_thd   capture basin              (SWING_CATCH_*)
%    K_si      4-vector LQR gains in SI (from lqr_design) — enables the
%              post-capture balance check
%    t_end     max sim time [s]
%
%  Success =  captured within t_end, cart stayed inside x_max, and (if K_si
%  given) the LQR holds |theta| < 15 deg for 3 s after handoff.

%% ---- Args ---------------------------------------------------------------
p = inputParser;
p.addParameter('fitfile', 'id_fit_result.mat');
p.addParameter('params', [], @isstruct);
p.addParameter('KE', 2.0);
p.addParameter('KX', 0.05);
p.addParameter('KXD', 0.02);
p.addParameter('KE_sweep', []);
p.addParameter('u_max', 0.6);
p.addParameter('x_max', 0.30);
p.addParameter('catch_th', 0.17);
p.addParameter('catch_thd', 1.5);
p.addParameter('K_si', []);
p.addParameter('t_end', 30);
p.parse(varargin{:});
a = p.Results;

if ~isempty(a.params), P = a.params;
else
    r = load(fullfile(fileparts(mfilename('fullpath')), a.fitfile));
    P = r.params;
end
g = 9.80665;

KEs = a.KE_sweep;  if isempty(KEs), KEs = a.KE; end

%% ---- Sweep the pump gain -------------------------------------------------
best = [];  runs = cell(1, numel(KEs));
fprintf('   KE      captured   t_catch   |x|max    |u|max\n');
for i = 1:numel(KEs)
    R = sim_swingup(P, KEs(i), a);
    runs{i} = R;
    fprintf('  %5.2f    %5s      %6.2fs   %.3f m   %.3f Nm %s\n', ...
        KEs(i), tern(R.captured, 'yes', 'NO'), R.t_catch, R.x_pk, R.u_pk, ...
        tern(R.ok, '', '  <-- fails budget'));
    if R.ok && (isempty(best) || R.t_catch < runs{best}.t_catch), best = i; end
end
if isempty(best)
    warning('swingup_design:none', ...
        'No KE in the sweep captures within budget — widen the sweep or raise u_max/x_max.');
    best = 1;
end
KE = KEs(best);  R = runs{best};
fprintf('\nSelected KE = %.2f  (capture %.2f s, |x|max %.3f m)\n', KE, R.t_catch, R.x_pk);

%% ---- Optional: LQR catch check ------------------------------------------
if ~isempty(a.K_si) && R.captured
    zc = R.z(end, :)';                       % state at handoff
    ctrl = @(z) max(-0.8, min(0.8, -a.K_si(:)' * z));
    ode  = @(t, z) nl_rhs(z, ctrl(z), P);
    sol  = ode45(ode, [0 3], zc, odeset('RelTol', 1e-6));
    caught = all(abs(sol.y(3,:)) < 0.26) && abs(sol.y(3,end)) < deg2rad(2);
    fprintf('LQR handoff from (th %.1f deg, thd %.2f rad/s): %s\n', ...
        rad2deg(zc(3)), zc(4), tern(caught, 'HOLDS — basin OK', ...
        'FAILS — shrink SWING_CATCH_* or retune the LQR'));
    R.lqr_holds = caught;
else
    fprintf('(pass ''K_si'' from lqr_design to verify the catch basin)\n');
end

%% ---- Plots ---------------------------------------------------------------
figure('Name', 'swingup\_design', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact');
nexttile; plot(R.t, rad2deg(wrap_pi(R.z(:,3))), 'LineWidth', 1); grid on;
ylabel('\theta [deg]'); yline(rad2deg(a.catch_th), 'g--'); yline(-rad2deg(a.catch_th), 'g--');
title(sprintf('Energy swing-up, KE = %.2f  (capture at %.2f s)', KE, R.t_catch));
nexttile; plot(R.t, R.E, 'LineWidth', 1); grid on;
ylabel('E [J]'); yline(0, 'g--'); yline(-2*g*P.ml, 'k:');
nexttile; plot(R.t, R.z(:,1), 'LineWidth', 1); grid on;
ylabel('x [m]'); yline(a.x_max, 'r--'); yline(-a.x_max, 'r--');
nexttile; plot(R.t, R.u, 'LineWidth', 1); grid on;
ylabel('u [Nm]'); xlabel('t [s]'); yline(a.u_max, 'r--'); yline(-a.u_max, 'r--');

%% ---- Paste-ready output ---------------------------------------------------
fprintf('\nPaste into CM7/Core/Inc/swingup_ctrl.h and rebuild M7:\n');
fprintf('#define SWING_ML_KGM          %.5ff   /* from id_fit */\n', P.ml);
fprintf('#define SWING_JT_KGM2         %.6ff   /* from id_fit */\n', P.Jt);
fprintf('#define SWING_KE              %.3ff\n', KE);
fprintf('#define SWING_KX              %.4ff\n', a.KX);
fprintf('#define SWING_KXD             %.4ff\n', a.KXD);

out = struct('KE', KE, 'run', R, 'params', P);

end

%% ======================================================================
%  Local helpers
%  ======================================================================

function R = sim_swingup(P, KE, a)
    g = 9.80665;
    x_mid = 0;                                     % centre = origin in sim
    % Firmware cart PD is in Nm/rev — convert to SI (x in m): gain / c.
    kx  = a.KX  / P.c;
    kxd = a.KXD / P.c;

    ctrl = @(z) clamp( ...
        KE * (0 - energy(z, P)) * z(4) * cos(z(3)) ...
        - kx * (z(1) - x_mid) - kxd * z(2), a.u_max);

    ode = @(t, z) nl_rhs(z, ctrl(z), P);
    ev  = @(t, z) catch_event(z, a.catch_th, a.catch_thd);
    z0  = [x_mid; 0; pi - 0.01; 0];                % hanging, tiny offset
    sol = ode45(ode, [0 a.t_end], z0, odeset('RelTol', 1e-6, 'Events', ev));

    zt = sol.y';  tt = sol.x';
    R.t = tt;  R.z = zt;
    R.u = arrayfun(@(k) ctrl(zt(k,:)'), 1:numel(tt))';
    R.E = arrayfun(@(k) energy(zt(k,:)', P), 1:numel(tt))';
    R.captured = ~isempty(sol.xe);
    R.t_catch  = tern(R.captured, tt(end), inf);
    R.x_pk     = max(abs(zt(:,1)));
    R.u_pk     = max(abs(R.u));
    R.ok       = R.captured && (R.x_pk < a.x_max);
end

function E = energy(z, P)
    g = 9.80665;
    E = 0.5 * P.Jt * z(4)^2 + g * P.ml * (cos(z(3)) - 1);
end

function [val, isterm, dir_] = catch_event(z, cth, cthd)
    inside = (abs(wrap_pi(z(3))) < cth) && (abs(z(4)) < cthd);
    val = 1 - 2*double(inside);   % crosses zero entering the basin
    isterm = 1;  dir_ = 0;
end

function dz = nl_rhs(z, u, P)
    [dz, ~] = cartpend_ode(0, z, u, P.Mt, P.ml, P.Jt, P.bc, P.Fc, P.bp, P.c);
end

function th = wrap_pi(th)
    th = mod(th + pi, 2*pi) - pi;
end

function v = clamp(v, lim), v = max(-lim, min(lim, v)); end

function s = tern(c, y, n), if c, s = y; else, s = n; end, end
