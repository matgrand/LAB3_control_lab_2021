%ass 2.3 
%position state space 

close all;
clear all;

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

%matrix A 
a31 = -k/(N*N*Jeq);
a32 = -a31;
a41 = k/Jb;
a42 = -a41;
a33 = -(1/Jeq)*(Beq+(Kt*Ke)/Req);
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

motor_sys = ss(A,B,C,D);


delta = (log(1/Mp))/(sqrt(pi^2+(log(1/Mp))^2));
wn = 3/(delta*ts5);
phi = atan(sqrt(1-delta^2)/delta);


%NOMINAL CONTROL
p1 = -wn*exp(+1i*phi);
p2 = -wn*exp(-1i*phi);
p3 = -wn*exp(+1i*phi/2);
p4 = -wn*exp(-1i*phi/2);

poles = 2*[p1 p2 p3 p4];

K_ss = real(place(A,B, poles)) 
K_ih = 0; %no integral action (but needed for the model)

F = [A B; C D];
Nxu = F\[0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4)]
Nu = Nxu(5)

%simulink model

step_start_time = 1;
enable_integral_action = 0; %disable integral action

step_height = 50; %deg

simulink_system = "ass2_3_SS";
open_system(simulink_system);
sim(simulink_system);

plot_scope(thh_scope, "Nominal SS regulator");



%ROBUST INTEGRAL ACTION

Ae = [0 C; zeros(4,1) A];
Be = [0; B];

p5 = -wn;

poles = 2*[p1 p2 p3 p4 p5];

Ke = acker(Ae, Be, poles);
K_ih = Ke(1)
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5)]

%simulink model

enable_integral_action = 1; 

sim(simulink_system);

plot_scope(thh_scope, "Robust SS regulator with integral action");











