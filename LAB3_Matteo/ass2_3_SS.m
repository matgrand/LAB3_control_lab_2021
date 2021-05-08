%ass 2.3 
%position state space 

close all;
clear all;

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

state_space_design_LQR;

%NOMINAL CONTROL
p1 = -wn*exp(+1i*phi);
p2 = -wn*exp(-1i*phi);
p3 = -wn*exp(+1i*phi/2);
p4 = -wn*exp(-1i*phi/2);

poles = 3*[p1 p2 p3 p4];

K_ss = real(place(A,B, poles)) 
K_ih = 0; %no integral action (but needed for the model)

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

poles = 3*[p1 p2 p3 p4 p5];

Ke = acker(Ae, Be, poles);
K_ih = Ke(1)
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5)]

%simulink model

enable_integral_action = 1; 

sim(simulink_system);

plot_scope(thh_scope, "Robust SS regulator with integral action");
































