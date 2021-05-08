%ass 2.4 (9)

% FREQUENCY DEPENDENT WEIGHT

clear all;
close all;

ts5 = 0.85; %settling time 
Mp = 0.30; %overshoot 

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

%LQR solution
Ka = lqr(Aa, Ba, Qa, Ra)
K_ss = Ka;
K_ih = 0; %for the model to work

%Nx Nu
F = [Aa Ba; Ca Da];
Nxu = F\[0; 0; 0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4); Nxu(5); Nxu(6)];
Nu = Nxu(7);


%simulink model

step_start_time = 1;
step_height = 50; %deg
enable_integral_action = 0; %no integral action

simulink_system = "model2_3_9_SS";
open_system(simulink_system);
 
sim(simulink_system);
plot_scope(thh_scope, "Frequency Dependent LQR regulator");

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

Qe = [qi zeros(1,6); zeros(6,1) Qa];
Re = Ra;

Ke = lqr(Ae,Be,Qe,Re);
K_ss = [Ke(2) Ke(3) Ke(4) Ke(5) Ke(6) Ke(7)];
K_ih = Ke(1);

%12
enable_integral_action = 1; %no integral action
sim(simulink_system);
plot_scope(thh_scope, "Frequency Dependent LQR regulator Integral Action");


%evaluate how much the resonant mode is attenuated
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


























