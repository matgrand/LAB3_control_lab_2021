%ass 2.4 
%STATE SPACE CONTROL WITH LQR METHODS

clear all;
close all;

load_param_resonance_load;
important_parameters;

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

%STATE SPACE
s = tf('s');

%real derivative transfer function for finding and filtering w
w_c = 2*pi*50;
delta_f = 1/sqrt(2);
H_w = (w_c^2*s)/(s^2+2*delta_f*w_c*s+w_c^2); %real derivative
[numH_w,denH_w] = tfdata(H_w, 'v'); %to be used it in a simulink block

%for findiing poles (maybe useless)
delta = (log(1/Mp))/(sqrt(pi^2+(log(1/Mp))^2));
wn = 3/(delta*ts5);
phi = atan(sqrt(1-delta^2)/delta);

%state space design
%matrix A 
a31 = -k/(N*N*Jeq);
a32 = -a31;
a33 = -(1/Jeq)*(Beq+(Kt*Ke)/Req);
a41 = k/Jb;
a42 = -a41;
a44 = -Bb/Jb;
A = [0 0 1 0; 
    0 0 0 1;
    a31 a32 a33 0;
    a41 a42 0 a44];

%B
b3 = (Kt*Kdrv)/(N*Jeq*Req);
B = [0;0; b3;0];

C = [1 0 0 0];
D = [0];

sysG = ss(A,B,C,D); %state space
sysGp = ss(-A, -B, C, D); %inverse state space

%Nx Nu
F = [A B; C D];
Nxu = F\[0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4)]
Nu = Nxu(5)

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
simulink_system = "ass2_3_SS";
open_system(simulink_system);

%2 validation
%NOMINAL LQR
enable_integral_action = 0; %disable integral action
step_height = 50; %deg

sim(simulink_system);
plot_scope(thh_scope, "Nominal LQR Regulator");


% 3 more general cost function
step_height = 50; %deg

dev_thh = 0.3*step_height*(pi/180); %maximum variation of the hub
dev_thd = pi/36; %maximum variation of the angle of the beam wrt the hub

dev_u = 10; %input maximum variation (to avoid saturation)

%Q = diag([])

























































