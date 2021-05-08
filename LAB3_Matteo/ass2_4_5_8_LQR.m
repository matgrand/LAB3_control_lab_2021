%ass 2.4 (5) Optional

clear all;
close all;

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

state_space_design_LQR;

%ROBUST INTEGRAL ACTION
Ae = [0 C; zeros(4,1) A];
Be = [0; B];
Ce = [1 0 0 0 0];

sysGe = ss(Ae,Be,Ce,D); %state space
sysGep = ss(-Ae, -Be, Ce, D); %inverse state space

%root locus analysis

r = 1/500; %can be tuned better

rlocus(sysGe*sysGep); %plot root locus
hold on
rl_poles = rlocus(sysGe*sysGep, 1/r);
plot(real(rl_poles), imag(rl_poles), 'rx', 'MarkerSize',10);
%points to plot lines
z0 = 0 +0j;
z1 = -70 +1j*tan(phi)*70;  %its 2*phi not phi %its tan(phi)
z2 = -70 -1j*tan(phi)*70;
phi_line = [z1, z0, z2];
plot(real(phi_line), imag(phi_line),'k');
z3 = -wn*delta +100i;
z4 = -wn*delta -100i;
sigma_line = [z3,z4];
plot(real(sigma_line), imag(sigma_line),'k');
xlim([-70 70]);
ylim([-70 70]);
hold off

K_ss_e = lqry(sysGe, 1 ,r);
K_ss = [K_ss_e(2) K_ss_e(3) K_ss_e(4) K_ss_e(5)]
K_ih = [K_ss_e(1)]

%6
step_start_time = 1;
step_height = 50; %deg
enable_integral_action = 1;

simulink_system = "model2_3_SS";
open_system(simulink_system);
 
sim(simulink_system);
plot_scope(thh_scope, "Robust LQR regulator with integral action");


% 7 optional

%LQR parameters
rho = 2.5; %balance between input and state (rho small -> input very costly)
dev_thh = 0.25*0.3*step_height*(pi/180); %maximum variation of the hub
dev_thd = (pi/36); %maximum variation of the angle of the beam wrt the hub

dev_u = 10; %input maximum variation (to avoid saturation)

q11_values = [1e-2 1e-1 1 1e1 1e2]; %try this values
q11 = 0.1;

%using Bryson's rule
Q = diag([q11 1/dev_thh^2 1/dev_thd^2 0 0]);
R = rho/dev_u^2;

K_ss_e = lqr(sysGe, Q, R);

K_ss = [K_ss_e(2) K_ss_e(3) K_ss_e(4) K_ss_e(5)]
K_ih = [K_ss_e(1)]

%8 validation 
%INTEGRAL ACTION LQR 
enable_integral_action = 1; %enable integral action
step_height = 50; %deg

sim(simulink_system);
plot_scope(thh_scope, "Integral Action LQR Regulator Custom Design");
































