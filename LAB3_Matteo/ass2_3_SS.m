%ass 2.3 
%position state space 

we_are_in_a_simulation = 0; %0 = WE ARE NOT IN A SIMULATION
stop_time = 10;

if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

state_space_design_LQR;

par.ts5 = ts5;
par.Mp = Mp;

%NOMINAL CONTROL
p1 = -wn*exp(+1i*phi);
p2 = -wn*exp(-1i*phi);
p3 = -wn*exp(+1i*phi/2);
p4 = -wn*exp(-1i*phi/2);

poles = 3*[p1 p2 p3 p4];
par.poles_nominal = poles;

K_ss = real(place(A,B, poles)) 
K_ih = 0; %no integral action (but needed for the model)

%simulink model

step_start_time = 2;
enable_integral_action = 0; %disable integral action
use_simple_observer = 1; %activate simple observer

step_height = 50; %deg

%open simulink model
if (we_are_in_a_simulation) 
    simulink_system = "model2_3_1_8_SS";
else
    simulink_system = "exp3_5_STATE_SPACE_LQR";
end

run_simulation;

plot_and_save(tmp,"Nominal_SS_Regulator",par);


%ROBUST INTEGRAL ACTION

Ae = [0 C; zeros(4,1) A];
Be = [0; B];

p1 = -wn*exp(+1i*phi/8);
p2 = -wn*exp(-1i*phi/8);
p3 = -wn*exp(+1i*phi/4);
p4 = -wn*exp(-1i*phi/4);

p5 = -wn;

poles = 2*[p1 p2 p3 p4 p5];
par.poles_robust = poles;

Ke = acker(Ae, Be, poles);
K_ih = Ke(1)
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5)]

%simulink model

enable_integral_action = 1; 

run_simulation;

plot_and_save(tmp,"Integral_action_SS_Regulator",par);
































