clear all;
load_params_resonantLoad;
SS_LRQ_commonStart;


%% Assignment 2.4.5 (robust control with integral action)

% design parameter
r = 1/1200;

% extended state with error (first component) [xi, thh, thd, wh, wd]
Ae = [0 C; ...
      zeros(4,1) A];
Be = [0; B];
Ce = [1 0 0 0 0];

% symmetric root locus
sysGe = ss(Ae,Be,Ce,D);
sysGep = ss(-Ae, -Be, Ce, D);

% plot root locus
% rlocus(sysGe*sysGep); 
% hold on
% 
% rl_poles = rlocus(sysGe*sysGep, 1/r);
% plot(real(rl_poles), imag(rl_poles), 'rx', 'MarkerSize', 10);
% 
% % plot allowable region of LHP
% a = 100; % random value, just to obtain points and draw lines
% max_sigma = -3/t_sett_5;
% z3 = max_sigma + 1i*a; 
% z4 = max_sigma - 1i*a;
% sigma_line = [z3,z4];
% plot(real(sigma_line), imag(sigma_line),'k');
% 
% z0 = 0 +0j;
% z1 = -a +1i*tan(phi)*a;
% z2 = -a -1i*tan(phi)*a;
% phi_line = [z1, z0, z2];
% plot(real(phi_line), imag(phi_line),'k');
% 
% xlim([-a a]);
% ylim([-a a]);
% hold off

% LQR matrix with robust control
Ke = lqry(sysGe, 1 ,r);
K_robust = [Ke(2) Ke(3) Ke(4) Ke(5)];
Ki_robust = Ke(1);

% only to run Simulink model
K = zeros(1,dimA);

%% Assignment 2.4.7 (robust control with integral action + Bryson's rule)

% since only the ratio matters, R constant
step_ref = 50;                             % [deg]
rho = 1;                                   % balance between input and state, x'Qx + u'(rho*R)u
dev_thh = 0.1*0.3*step_ref*(pi/180);       % maximum theta hub (thh) error
dev_thd = pi/36;                           % maximum moduli of theta displacement (thd)
dev_u = 10;                                % input maximum variation (to avoid saturation)

% weight of the integrator state
% try {1e-2, 1e-1, 1, 1e1, 1e2}
q11 = 0.01;

% recall: state vector [xi, thh, thd, wh, wd]
Q = diag([q11, 1/dev_thh^2, 1/dev_thd^2, 0, 0]);
R = rho * (1/dev_u^2);

[Ke,S,e] = lqr(sysGe, Q, R);
K_robust = [Ke(2) Ke(3) Ke(4) Ke(5)];
Ki_robust = Ke(1);