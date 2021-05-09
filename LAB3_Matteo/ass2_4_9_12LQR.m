%ass 2.4 (9)

% FREQUENCY DEPENDENT WEIGHT

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

%resonant frequency of the elastic joint

sysG = ss(A,B,C,D); %state space

eig_A = eig(A); %eigenvalues
poles_G = pole(sysG);

w0 = imag(eig_A(2)); 

%PARAMETERS %%%%%%%%%%%%%%%%%%%%%%
dev_thh = 5*pi/180;
dev_u = 10;
q22 = 300; % 0.01 1 100
r = 1.2/dev_u^2; 

par.dev_thh = dev_thh;
par.dev_u = dev_u;
par.q22 = q22;
par.r = r;

%realization of Htq
A1q = [0 1; -w0^2 0];
B1q = [0;1];
C1q = [sqrt(q22)*w0^2 0];
D1q = 0;

%realization of Hq
Aq = A1q;
Bq = [zeros(2,1) B1q zeros(2,2)];
Cq = [zeros(1,2); C1q; zeros(2,2)];
Dq = diag([1/dev_thh D1q 0 0]); %on the assignment is diag([1/dev_thh 0 0 0]);

%augmented state sysA (x_a = [x; x_q])
Aa = [A zeros(4,2); Bq Aq];
Ba = [B;0;0];
Ca = [C zeros(1,2)];
Da = 0;

%cost matrices
Qa = [Dq.'*Dq Dq.'*Cq; Cq.'*Dq Cq.'*Cq];
Na = 0;
Ra = r;

par.Qa = Qa;
par.Na = Na;
par.Ra = Ra;

%LQR solution
Ka = lqr(Aa, Ba, Qa, Ra)
K_ss = Ka;
K_ih = 0; %for the model to work

par.K_ss = K_ss;

%Nx Nu
F = [Aa Ba; Ca Da];
Nxu = F\[0; 0; 0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4); Nxu(5); Nxu(6)];
Nu = Nxu(7);


%simulink model
if (we_are_in_a_simulation) 
    simulink_system = "model2_3_9_12_SS";
else
    simulink_system = "exp3_5_STATE_SPACE_LQR_FREQ";
end

step_start_time = 2;
step_height = 50; %deg
enable_integral_action = 0; %no integral action

run_simulation;
plot_and_save(ans, "Frequency_Dependent_LQR_regulator",par);

%evaluate how much the resonant mode is attenuated
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%11 iNTEGRAL ACTION

Ae = [0 Ca; zeros(6,1) Aa];
Be = [0; Ba];
Ce = [0 Ca];

%PARAMETERS %%%%%%%%%%%%%%%%%%%%%%
dev_thh = 5*pi/180;
dev_u = 10;
q22 = 300; % 0.01 1 100
r = 1.2/dev_u^2;
qi = 10;

par.dev_thh = dev_thh;
par.dev_u = dev_u;
par.q22 = q22;
par.r = r;
par.qi = qi;

Qe = [qi zeros(1,6); zeros(6,1) Qa];
Re = Ra;

par.Qe = Qe;
par.Re = Re;

Ke = lqr(Ae,Be,Qe,Re);
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5) Ke(6) Ke(7)];
K_ih = Ke(1);

par.K_ss = K_ss;
par.K_ih = K_ih;

%12
enable_integral_action = 1; %no integral action

run_simulation;
plot_and_save(ans, "Frequency_Dependent_LQR_regulator_integral_action",par);

%evaluate how much the resonant mode is attenuated
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


























