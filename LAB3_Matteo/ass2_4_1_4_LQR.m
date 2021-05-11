%ass 2.4 
%STATE SPACE CONTROL WITH LQR METHODS
we_are_in_a_simulation = 0; %0 = WE ARE NOT IN A SIMULATION
stop_time = 10;


if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 
par.ts5 = ts5;
par.Mp = Mp;

state_space_design_LQR;

sysG = ss(A,B,C,D); %state space
sysGp = ss(-A, -B, C, D); %inverse state space

%root locus analysis
r = 1/3850; %tuned value
par.r = r;
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
par.K_ss = K_ss;

%simulink model
step_start_time = 2;
use_simple_observer = 1; %activate simple observer
if (we_are_in_a_simulation) 
    simulink_system = "model2_3_1_8_SS";
else
    simulink_system = "exp3_5_STATE_SPACE_LQR";
end

%2 validation
%NOMINAL LQR
enable_integral_action = 0; %disable integral action
step_height = 50; %deg

run_simulation;
plot_and_save(tmp, "Nominal_LQR_Regulator_Root_Locus",par);

% 3 more general cost function
step_height = 50; %deg

rho = 1.2; %balance between input and state (rho big -> input very costly)
dev_thh = 0.7*0.3*step_height*(pi/180); %maximum variation of the hub
dev_thd = (pi/36); %maximum variation of the angle of the beam wrt the hub

dev_u = 10; %input maximum variation (to avoid saturation)

%using Bryson's rule
Q = diag([1/dev_thh^2 1/dev_thd^2 0 0]);
R = rho/dev_u^2;

par.rho = rho;
par.dev_thh = dev_thh;
par.dev_thd = dev_thd;
par.dev_u = dev_u;
par.Q = Q;
par.R = R;

K_ss = lqr(sysG, Q, R);
par.K_ss = K_ss;

%4 validation 
%NOMINAL LQR 
enable_integral_action = 0; %disable integral action
step_height = 50; %deg

run_simulation;
plot_and_save(tmp, "Nominal_LQR_Regulator_Custom_Design_1",par);































































