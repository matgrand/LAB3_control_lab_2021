%ass 2.2 1
%position pid for the hub position (collocated control)

we_are_in_a_simulation = 0; %0 = WE ARE NOT IN A SIMULATION
stop_time = 10;

if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

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
alpha = 4;
wgc_mult = 10;

[Kp,Ki,Kd,Tl,Kw, num_PID, den_PID] = PID_designer_CT(P_u_thh, ts_5, Mp , wgc_mult, alpha)

PID_param.Kp = Kp;
PID_param.Ki = Ki;
PID_param.Kd = Kd;
PID_param.Tl = Tl;
PID_param.Kw = Kw;

%open simulink model
if (we_are_in_a_simulation) 
    simulink_system = "model2_2_PID";
else
    simulink_system = "exp3_3_PID";
end

step_time = 2; %yes, they are different
step_height = 50; %deg
enable_anti_windup = 0;

run_simulation;
plot_and_save(tmp, "PID_standard_50deg",PID_param);

step_height = 120; %deg
enable_anti_windup = 0;
run_simulation;
plot_and_save(tmp, "PID_standard_120deg",PID_param);

step_height = 50; %deg
enable_anti_windup = 1;
run_simulation;
plot_and_save(tmp, "PID_anti_windup_50deg",PID_param);

step_height = 120; %deg
enable_anti_windup = 1;
run_simulation;
plot_and_save(tmp, "PID_anti_windup_120deg",PID_param);














