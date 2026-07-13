function K_fw = lqr_design(varargin)
%LQR_DESIGN  Design + verify stage-1 balance gains from the fitted model.
%
%  Usage (after id_fit has produced id_fit_result.mat):
%    K_fw = lqr_design();                       % defaults
%    K_fw = lqr_design('Q', diag([60 5 800 2]), 'R', 20);
%    K_fw = lqr_design('fitfile', 'id_fit_result.mat', 'u_max', 0.8, ...
%                      'x_max', 0.30, 'theta0_deg', 2:2:10);
%
%  Pipeline:
%    1. Linearize the fitted cart+link model about upright (Coulomb term
%       dropped — not linearizable; it stays in the verification sim, per
%       Eltohamy & Kuo who found exactly this mismatch breaks controllers).
%    2. K = lqr(A, B, Q, R) in SI states [x_m, xd_m, th, thd] -> torque.
%    3. Verify on the FULL nonlinear model (cartpend_ode, friction included)
%       with the firmware torque clamp, sweeping initial link offsets.
%       Success = never crosses the 15 deg firmware abort, settles upright,
%       stays inside the rail budget.
%    4. Print the firmware gains and the paste-ready BAL_K_BASE line
%       (converted to firmware units: x in motor rev -> K1,K2 scaled by c).
%
%  Q philosophy (paper): weight POSITIONS heavily, velocities lightly.
%  Raise Q(3,3) if the link wanders; raise Q(1,1) if the cart drifts; raise
%  R if u saturates in the sweep.
%
%  MEASUREMENT DELAYS — NOT MODELED HERE (arch review 2026-07-11, F13).
%  Neither the linearization nor the nonlinear sweep includes the firmware's
%  measurement lag; the design must carry enough phase margin to absorb it:
%    - theta_dot: ~7 ms total (2nd-order Butterworth 40 Hz group delay
%      ~5.6 ms + 1 kHz FD half-sample + AS5047P pipeline; partly offset by
%      the chip's DAEC). This is the documented firmware budget.
%    - x, x_dot: up to ~4 ms sample age from the ODrive CAN encoder
%      broadcast (configured >=250 Hz, U2) PLUS the ODrive's own velocity
%      estimator lag — unmeasured until bring-up.
%    - u: zero-order hold, torque recomputed at 1 kHz (adds up to 1 ms).
%  For the stage-1 single link (closed-loop poles ~ a few rad/s) ~10 ms of
%  loop delay costs only a few degrees of phase — fine IF the sweep passes
%  with margin. If a design only *marginally* passes the sweep (u_pk at the
%  clamp, slow settling), do NOT paste it: it has no budget left for these
%  delays. Either soften Q / raise R, or re-verify with the delay made
%  explicit (e.g. run the sweep's controller on states delayed by ~7-10 ms).

%% ---- Args ---------------------------------------------------------------
p = inputParser;
p.addParameter('fitfile', 'id_fit_result.mat');
p.addParameter('params',  [], @isstruct);          % bypass the file
p.addParameter('Q', diag([60 5 800 2]), @(v) isequal(size(v), [4 4]));
p.addParameter('R', 20, @(v) isscalar(v) && v > 0);
p.addParameter('u_max', 0.8, @isscalar);           % = BAL_TORQUE_MAX_NM
p.addParameter('x_max', 0.30, @isscalar);          % cart excursion budget [m]
p.addParameter('theta0_deg', 2:2:10);
p.addParameter('t_end', 6.0);
p.parse(varargin{:});
a = p.Results;

if ~isempty(a.params)
    P = a.params;
else
    fdir = fileparts(mfilename('fullpath'));
    r = load(fullfile(fdir, a.fitfile));
    P = r.params;
end

%% ---- Linearize about upright --------------------------------------------
g  = 9.80665;
M0 = [P.Mt, P.ml; P.ml, P.Jt];             % mass matrix at th = 0
Kq = [0, 0; 0, g*P.ml];                    % d(rhs)/d[x, th]
Cq = [P.bc, 0; 0, P.bp];                   % d(rhs)/d[xd, thd] (viscous only)
Bf = [2*pi/P.c; 0];                        % torque -> generalized force

Mi = M0 \ eye(2);
% States z = [x_m, xd_m, th, thd]; accelerations qdd = Mi*(Kq*q - Cq*qd + Bf*u)
A = [0 1 0 0;
     [Mi(1,:)*Kq(:,1), -Mi(1,:)*Cq(:,1), Mi(1,:)*Kq(:,2), -Mi(1,:)*Cq(:,2)];
     0 0 0 1;
     [Mi(2,:)*Kq(:,1), -Mi(2,:)*Cq(:,1), Mi(2,:)*Kq(:,2), -Mi(2,:)*Cq(:,2)]];
B = [0; Mi(1,:)*Bf; 0; Mi(2,:)*Bf];

ol = eig(A);
fprintf('Open-loop poles: %s  (one positive real = inverted, correct)\n', ...
        sprintf('%.2f%+.2fi  ', [real(ol) imag(ol)]'));

%% ---- LQR ----------------------------------------------------------------
[K_si, ~, cl] = lqr(A, B, a.Q, a.R);
fprintf('Closed-loop poles: %s\n', sprintf('%.2f%+.2fi  ', [real(cl) imag(cl)]'));

%% ---- Nonlinear verification sweep ---------------------------------------
th0s = a.theta0_deg(:)' * pi/180;
n    = numel(th0s);
res  = struct('ok', false(1,n), 'u_pk', zeros(1,n), 'dx_pk', zeros(1,n));
sols = cell(1,n);

ctrl = @(z) max(-a.u_max, min(a.u_max, -K_si * z));   % clamped LQR
ode  = @(t, z) nl_rhs(z, ctrl(z), P);

for i = 1:n
    z0 = [0; 0; th0s(i); 0];
    sol = ode45(ode, [0 a.t_end], z0, odeset('RelTol', 1e-6));
    zt  = sol.y';  tt = sol.x';
    ut  = arrayfun(@(k) ctrl(zt(k,:)'), 1:numel(tt));
    res.u_pk(i)  = max(abs(ut));
    res.dx_pk(i) = max(abs(zt(:,1)));
    res.ok(i) = all(abs(zt(:,3)) < 0.26) ...            % firmware abort bound
             && abs(zt(end,3)) < deg2rad(1) ...
             && res.dx_pk(i) < a.x_max;
    sols{i} = struct('t', tt, 'z', zt, 'u', ut(:));
    fprintf('  th0 = %4.1f deg : %s   |u|max %.3f Nm   |x|max %.3f m\n', ...
            th0s(i)*180/pi, tern(res.ok(i), 'OK  ', 'FAIL'), ...
            res.u_pk(i), res.dx_pk(i));
end

figure('Name', 'lqr\_design verification', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile; hold on; grid on;
for i = 1:n, plot(sols{i}.t, rad2deg(sols{i}.z(:,3))); end
yline( 15, 'r--'); yline(-15, 'r--'); ylabel('\theta [deg]');
title(sprintf('Nonlinear closed loop, u_{max} = %.1f Nm', a.u_max));
nexttile; hold on; grid on;
for i = 1:n, plot(sols{i}.t, sols{i}.z(:,1)); end
yline(a.x_max, 'r--'); yline(-a.x_max, 'r--'); ylabel('x [m]');
nexttile; hold on; grid on;
for i = 1:n, plot(sols{i}.t, sols{i}.u); end
yline(a.u_max, 'r--'); yline(-a.u_max, 'r--'); ylabel('u [Nm]'); xlabel('t [s]');

%% ---- Firmware gains ------------------------------------------------------
% Firmware states are [x_rev, xd_rev, th, thd] and it computes
%   u = -(K0*dx + K1*xd + K2*th + K3*thd)      (balance_ctrl.c)
% x_m = c * x_rev  ->  scale the two cart gains by c.
K_fw = [K_si(1)*P.c, K_si(2)*P.c, K_si(3), K_si(4)];

fprintf('\nK (SI, torque =  -K*[x m, xd m/s, th rad, thd rad/s]):\n   [%.4f  %.4f  %.4f  %.4f]\n', K_si);
fprintf('K (firmware units, x in motor rev):\n   [%.4f  %.4f  %.4f  %.4f]\n', K_fw);
if ~all(res.ok)
    warning('lqr_design:sweep', ...
        'Sweep has failures — retune Q/R before flashing these gains.');
end
fprintf('\nPaste into CM7/Core/Inc/balance_ctrl.h and rebuild M7:\n');
fprintf('#define BAL_K_BASE            { %.4ff, %.4ff, %.4ff, %.4ff }\n', K_fw);
fprintf('\nReminders: STATE_EST_THETA_SIGN verified on bench? Torque clamp %.1f Nm ok for these |u| peaks?\n', a.u_max);

end

%% ======================================================================
%  Local helpers
%  ======================================================================

function dz = nl_rhs(z, u, P)
    % Full nonlinear plant (shared model), Coulomb + viscous friction in.
    [dz, ~] = cartpend_ode(0, z, u, P.Mt, P.ml, P.Jt, P.bc, P.Fc, P.bp, P.c);
end

function s = tern(cond, y, n)
    if cond, s = y; else, s = n; end
end
