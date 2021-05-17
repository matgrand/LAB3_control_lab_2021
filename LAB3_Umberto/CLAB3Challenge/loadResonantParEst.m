%% ATTENTION ppr "MOTORE 8"  and "MOTORE 10"
% for other motors sens.enc.deg2pulse = 500*4
% for "MOTORE8" and "MOTORE 10" sens.enc.deg2pulse = 1024*4
load_params_resonant_case;

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