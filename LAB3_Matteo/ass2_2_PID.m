%ass 2.2 1
%position pid for the hub position (collocated control)

clear all;

%load parameters
load_param_resonance_load;
important_parameters;

%tf from u to thh

s = tf('s');

Jeq = Jm+Jh/N^2;
D_dot_tau = Jeq*Jb*s^3+(Jeq*Bb+Jb*Beq)*s^2 + (Beq*Bb+k*(Jeq+Jb/N^2))*s+k*(Beq+Bb/N^2);

P_u_thh = (Kdrv*Kt*(Jb*s^2+Bb*s+k)) / ((N*s)*(Req*D_dot_tau+Kt*Ke*(Jb*s^2+Bb*s+k)));


%PID design
ts_5 = 0.85;
Mp = 0.3;
alpha = 5;
wgc_mult = 10;

[Kp,Ki,Kd,Tl,Kw, num_PID, den_PID] = PID_designer_CT(P_u_thh, ts_5, Mp , wgc_mult, alpha)


%open simulink model

step_height = 50; %deg
enable_anti_windup = 0;

simulink_system = "model2_2_PID";
open_system(simulink_system);

sim(simulink_system);
plot_scope(thh_scope,"Standard PID, 50 deg step");

step_height = 120; %deg
enable_anti_windup = 0;
sim(simulink_system);
plot_scope(thh_scope,"Standard PID, 120 deg step");

step_height = 50; %deg
enable_anti_windup = 1;
sim(simulink_system);
plot_scope(thh_scope,"PID with Anti-Windup, 50 deg step");

step_height = 120; %deg
enable_anti_windup = 1;
sim(simulink_system);
plot_scope(thh_scope,"PID with Anti-Windup, 120 deg step");















