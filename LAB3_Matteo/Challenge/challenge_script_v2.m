%challenge

close all;
clear all;

%see ass 2.4 (5)-(8)

we_are_in_a_simulation = 1; %0 = WE ARE NOT IN A SIMULATION

step_start_time = 1;
step_height = 50; %deg

if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

%% Frequency Shaped LQR Integral Action

% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stop_time = 3;
enable_integral_action = 1; %enable integral action
enable_anti_windup = 1; %anti windup for the integral action

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

%freq shaped in
dev_thh = 0.25*5/180*pi;   %0.25       %deviation on the thh component
dev_thd = 0.585*pi/36;     %0.585
dev_u = sqrt(10);                         %input deviation
      
r = 1/dev_u^2; %0.1                       %input weight
qi = 0.1; %0.1                  %integral weight
dq1 = 1/dev_thh^2; %1/dev_thh^2                  %thh weight
dq2 = 1/dev_thd^2; %1/dev_thd^2                 %thd weight
dq3 = 6.6; %6.6                  %wh hub angular velocity
dq4 = 0; %0                 %wd beam angular velocity

Kw = 0.092; %0.092 %anti_windup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

par.ts5 = ts5;
par.Mp = Mp;
par.Kw = Kw;
par.dev_thh = dev_thh;
par.dev_u = dev_u;
par.r = r;
par.qi = qi;

state_space_design_LQR;

%resonant frequency of the elastic joint
sysG = ss(A,B,C,D); %state space
eig_A = eig(A); %eigenvalues
w0 = imag(eig_A(2)); 

w0 = 41.312420865606860; %from umbe, controlli automatici book

%realization of Htq
A1q = [0 1; -w0^2 0];
B1q = [0;1];
C1q = [sqrt(dq2)*w0^2 0];
D1q = 0;

%realization of Hq
Aq = A1q; %ok
Bq = [zeros(2,1) B1q zeros(2,2)];
Cq = [zeros(1,2); C1q; zeros(2,2)];
Dq = diag([1/dev_thh, D1q, sqrt(dq3), sqrt(dq4)]); %on the assignment is diag([1/dev_thh 0 0 0]);

%augmented state sysA (x_a = [x; x_q])
Aa = [A zeros(4,2); Bq Aq]; %ok
Ba = [B;0;0];
Ca = [C zeros(1,2)];
Da = 0;

%cost matrices
Qa = [Dq'*Dq,Dq'*Cq;...
      Cq'*Dq,Cq'*Cq];
Na = 0;
Ra = r;

%Nx Nu
F = [Aa Ba; Ca Da];
Nxu = F\[0; 0; 0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4); Nxu(5); Nxu(6)];
Nu = Nxu(7);

%11 iNTEGRAL ACTION

Ae = [0 Ca; zeros(6,1) Aa];
Be = [0; Ba];
Ce = [0 Ca];

Qe = blkdiag(qi,Qa);
Re = Ra;

par.Qe = Qe;
par.Re = Re;

Ke = lqr(Ae,Be,Qe,Re);
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5) Ke(6) Ke(7)];
K_ih = Ke(1);

par.K_ss = K_ss;
par.K_ih = K_ih;


% copying UMBE

%Qe = diag([0.1, 1.6869e3, 0, 6, 1, 3.8250e8])


%% TRICK
%0
enable_trick = 0;

t0 = step_start_time;
t1 = t0 + 0.125;
t2 = t1 + 0;

%% Simulation
if (we_are_in_a_simulation) 
    simulink_system = "challenge_model_v2";
else
    disp('ERROR: WE ARE NOT IN A SIMULATION AND WE SHOULD BE');
    simulink_system = "NONE";
end

use_simple_observer = 1; %activate simple observer

%8 validation 
%INTEGRAL ACTION LQR 

run_simulation;
plot_and_save(tmp, "Challenger", par);

t = thh_scope1.time;
thh = thh_scope1.signals(2).values;
ref = thh_scope1.signals(1).values;

S = stepinfo(thh, t, step_height, 'SettlingTimeThreshold', 0.05)
%fprintf('Settling Time: \n %f \n', S.SettlingTime-step_start_time);
%fprintf('Overshoot:\n %f \n', S.Overshoot-step_start_time);

close all;

figure;
plot(t,thh);
hold on;
plot(t, ref,'--c');
ts_thrs_pos = (step_height+0.05*step_height)+ zeros(size(t));
plot(t, ts_thrs_pos, '--r');
ts_thrs_neg = (step_height-0.05*step_height) + zeros(size(t));
plot(t, ts_thrs_neg,'--r');
xline(S.SettlingTime, '--');
ylabel('thh[deg]');
xlabel('t[s]');
legend('Step response 50[deg]', 'Reference', '+ 5% threshold', '- 5% threshold', 'Settling time');
















