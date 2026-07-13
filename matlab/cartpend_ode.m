function [dx, y] = cartpend_ode(t, x, u, Mt, ml, Jt, bc, Fc, bp, c, varargin)
%CARTPEND_ODE  Cart + single inverted link dynamics (grey-box / simulation).
%
%  Shared by id_fit.m (nlgreyest FileName) and lqr_design.m (nonlinear
%  closed-loop verification) so the fitted model IS the design model.
%
%  States  x = [x_m; xd_m; th; thd]
%     x_m   cart position [m]           th   link angle from UPRIGHT [rad]
%     xd_m  cart velocity [m/s]         thd  link rate [rad/s]
%  Input   u = motor torque [Nm]  (belt force F = u * 2*pi / c)
%  Outputs y = [x_m; th]  (the two directly-measured channels)
%
%  Parameters (SI, lumped — only these combinations are identifiable):
%     Mt  total translating mass M + m               [kg]
%     ml  first mass moment of the link, m*l         [kg m]
%     Jt  link inertia about the pivot, J + m*l^2    [kg m^2]
%     bc  viscous cart friction                      [N s/m]
%     Fc  Coulomb cart friction (tanh-smoothed)      [N]
%     bp  viscous pivot friction                     [N m s/rad]
%     c   cart travel per motor rev (FIXED, measured)[m/rev]
%
%  EOM (theta from upright, positive toward +x — matches state_est.c):
%     [Mt        ml*cos(th)] [xdd ]   [F + ml*sin(th)*thd^2 - bc*xd - Fc*tanh(xd/vs)]
%     [ml*cos(th)    Jt    ] [thdd] = [g*ml*sin(th) - bp*thd                        ]
%
%  Friction model per Eltohamy & Kuo (1997): viscous + Coulomb; Coulomb is
%  smoothed with tanh so nlgreyest's gradients behave (vs = 0.01 m/s).

g  = 9.80665;
vs = 0.01;                          % Coulomb smoothing velocity [m/s]

xd  = x(2);
th  = x(3);
thd = x(4);

F = u(1) * 2*pi / c;                % motor torque -> belt force on cart

% Mass matrix and right-hand side
m11 = Mt;            m12 = ml*cos(th);
m21 = m12;           m22 = Jt;
r1  = F + ml*sin(th)*thd^2 - bc*xd - Fc*tanh(xd/vs);
r2  = g*ml*sin(th) - bp*thd;

det_ = m11*m22 - m12*m21;
xdd  = ( m22*r1 - m12*r2) / det_;
thdd = (-m21*r1 + m11*r2) / det_;

dx = [xd; xdd; thd; thdd];
y  = [x(1); th];

end
