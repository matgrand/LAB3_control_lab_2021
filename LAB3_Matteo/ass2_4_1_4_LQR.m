%ass 2.4 
%STATE SPACE CONTROL WITH LQR METHODS

clear all;
close all;

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

state_space_design_LQR;

sysG = ss(A,B,C,D); %state space
sysGp = ss(-A, -B, C, D); %inverse state space

%root locus analysis
r = 1/3850; %tuned value
rlocus(sysG*sysGp); %plot root locus
hold on
rl_poles = rlocus(sysG*sysGp, 1/r);
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

%controller
K_ss = lqry(sysG,1,r)
K_ih = 0; %needed for the model (no integral action here)


%simulink model
step_start_time = 1;
simulink_system = "model2_3_SS";
open_system(simulink_system);

%2 validation
%NOMINAL LQR
enable_integral_action = 0; %disable integral action
step_height = 50; %deg

sim(simulink_system);
plot_scope(thh_scope, "Nominal LQR Regulator Root Locus");


% 3 more general cost function
step_height = 50; %deg

rho = 100; %balance between input and state (rho small -> input very costly)
dev_thh = 0.25*0.3*step_height*(pi/180); %maximum variation of the hub
dev_thd = 1*(pi/36); %maximum variation of the angle of the beam wrt the hub

dev_u = 10; %input maximum variation (to avoid saturation)

%since our current ss uses thb and thh we need to switch to thd (relative position of the beam)

T = [1 0 0 0; 1 1 0 0; 0 0 1 0; 0 0 1 1];

A_1 = T\A*T;
B_1 = T\B;
C_1 = C*T;
D_1 = D;

sysG_1 = ss(A_1, B_1, C_1, D_1);

%using Bryson's rule
Q = diag([1/dev_thh^2 1/dev_thd^2 0 0]);
R = rho/dev_u^2;

K_ss_1 = lqr(sysG_1, Q, R);
K_ss = K_ss_1*T; %switch back to original system

%maybe it's correct this way, but it doesn't make sense
%K_ss = lqr(sysG, Q, R);

%4 validation 
%NOMINAL LQR 
enable_integral_action = 0; %disable integral action
step_height = 50; %deg

sim(simulink_system);
plot_scope(thh_scope, "Nominal LQR Regulator Custom Design 1");
































































