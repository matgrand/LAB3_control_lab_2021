%parameters

Jh = 6.84e-4; %<-Aluminium %5.1e-4 <-Quanser %hub moment of inertia

Bh = 0; %it's whatever, we use Beq

tausf = 0.0125; % 0.0057; %estimated in previous labs

Jb = 1.4e-3; %<-Aluminium %2.0e-3 <-Quanser %Beam moment of inertia

Bb = 3.4e-3; %Beam viscous friction coefficient %to be estimated

k = 0.83; %Joint stiffness %to be estimated


%other important parameters

Beq = 1.067548701517996e-06;  % 1.2195e-06; %same estimate of prev labs

Jeq = mot.J + Jh/gbox.N^2;
%Jeq = 6.790316162587374e-07;

Ts = 0.001; %Sample time


%renaming things for readability

Kdrv = drv.dcgain;
Tdrv = drv.Tc;
Kt = mot.Kt;
Ke = mot.Ke;
La = mot.L;
Rs = sens.curr.Rs;
Ra = mot.R;
Req = Ra + Rs;
Jm = mot.J;
N = gbox.N;

Bb_est = 0.0043; %%%%%%%%%%%%%%%%%%%%%% CHANGE IT WITH THE ESTIMATED ONE
k_est = 0.8490;   %%%%%%%%%%%%%%%%%%% CHANGE IT WITH THE ESTIMATED ONE

%k = k_est;
%Bb = Bb_est;

%% black box estimate (umbe)

BeqEst = 1.067548701517996e-06;
dampingFactorEst = 0.094826604549665;
JeqEst = 6.790316162587374e-07;
tauSfEst = 0.012480599355650;
wnEst = 25.025907269895335;



mld.tausf = tauSfEst;
%Beq = BeqEst;
Req = mot.R + sens.curr.Rs;
mld.wn = wnEst;
mld.k = mld.Jb*mld.wn^2;
mld.Bb = mld.Jb*(2*dampingFactorEst*mld.wn);
%mld.Jh = 6.84e-4 alumininum
%mld.Jb = 1.4e-3 alumininum
%mld.Jh = 5.1e-4; %Quanser
%mld.Jb = 2.0e-3; %Quanser
Ts = 0.001;
Beq = mot.B+(mld.Bb/gbox.N1^2);
Jeq = mot.J+(mld.Jh/gbox.N1^2);