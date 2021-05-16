%challenge

close all;
clear all;

%see ass 2.4 (5)-(8)

we_are_in_a_simulation = 1; %0 = WE ARE NOT IN A SIMULATION
stop_time = 2;
step_height = 50; %deg

if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

%% PARAMETERS
ts5 = 0.15; %settling time 
Mp = 0.80; %overshoot 
par.ts5 = ts5;
par.Mp = Mp;

%% STATE SPACE DESIGN
state_space_design_LQR;

%extended state space
Ae = [0 C; zeros(4,1) A];
Be = [0; B];
Ce = [1 0 0 0 0];

sysGe = ss(Ae,Be,Ce,D); %state space

%% Custom Design LQR
% 7 optional

%LQR parameters
rho = 1; %balance between input and state (rho small -> input very costly)
dev_thh = 1   *0.3*step_height*(pi/180); %maximum variation of the hub
dev_thd = 1     *(pi/36); %maximum variation of the angle of the beam wrt the hub

dev_u = 10; %input maximum variation (to avoid saturation)

q11_values = [1e-2 1e-1 1 1e1 1e2]; %try this values
q11 = 0.1;

Kw = 5;


%using Bryson's rule
Q = diag([q11 1/dev_thh^2 1/dev_thd^2 0 0]);
R = rho/dev_u^2;

par.rho = rho;
par.dev_thh = dev_thh;
par.dev_thd = dev_thd;
par.dev_u = dev_u;
par.q11 = q11;
par.Q = Q;
par.R = R;

K_ss_e = lqr(sysGe, Q, R);

K_ss = [K_ss_e(2) K_ss_e(3) K_ss_e(4) K_ss_e(5)];
K_ih = [K_ss_e(1)];

par.K_ss = K_ss;
par.K_ih = K_ih;


%% Simulation
if (we_are_in_a_simulation) 
    simulink_system = "challenge_model";
else
    disp('ERROR: WE ARE NOT IN A SIMULATION AND WE SHOULD BE');
    simulink_system = "exp3_5_STATE_SPACE_LQR";
end

step_start_time = 1;
use_simple_observer = 1; %activate simple observer

%8 validation 
%INTEGRAL ACTION LQR 
enable_integral_action = 1; %enable integral action
enable_anti_windup = 1; %anti windup for the integral action

run_simulation;
plot_and_save(tmp, "Challenger", par);

t = thh_scope1.time;
thh = thh_scope1.signals(2).values;
ref = thh_scope1.signals(1).values;

S = stepinfo(thh, t, step_height, 'SettlingTimeThreshold', 0.05);
fprintf('Settling Time: \n %f \n', S.SettlingTime-step_start_time);
fprintf('Overshoot:\n %f \n', S.Overshoot-step_start_time);

close all;

figure;
plot(t,thh);
hold on;
plot(t, ref);
ts_thrs_pos = (step_height+0.05*step_height)+ zeros(size(t));
plot(t, ts_thrs_pos);
ts_thrs_neg = (step_height-0.05*step_height) + zeros(size(t));
plot(t, ts_thrs_neg);
xline(S.SettlingTime);


















