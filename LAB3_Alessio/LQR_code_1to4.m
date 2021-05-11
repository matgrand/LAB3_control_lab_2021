clear all;
load_params_resonantLoad;
SS_LRQ_commonStart;


%% Assignment 2.4.1 (basic LQR)

% design parameter
r = 1/3000; %3850

% symmetric root locus
sysG = ss(A,B,C,D);
sysGp = ss(-A, -B, C, D);

% plot root locus
% rlocus(sysG*sysGp);
% hold on
% 
% rl_poles = rlocus(sysG*sysGp, 1/r);
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

% LQR matrix
K = lqry(sysG,1,r);

% only to run Simulink model
K_robust = zeros(1,dimA);
Ki_robust = 0;

%% Assignment 2.4.3 (more general cost function with Bryson's rule)

step_ref = 50;                             % [deg]
rho = 1;                                   % balance between input and state, x'Qx + u'(rho*R)u
dev_thh = 0.3*step_ref*(pi/180);           % maximum theta hub (thh) error
dev_thd = pi/36;                           % maximum moduli of theta displacement (thd)
dev_u = 10;                                % input maximum variation (to avoid saturation)

% recall: state vector [thh, thd, wh, wd]
Q = diag([1/dev_thh^2, 1/dev_thd^2, 0, 0]);
R = rho * (1/dev_u^2);

[K,S,e] = lqr(sysG, Q, R);