%parameters

Jh = 6.84e-4; %<-Aluminium %5.1e-4 <-Quanser %hub moment of inertia

Bh = 0; %it's whatever, we use Beq

tausf = 0.0125; % 0.0057; %estimated in previous labs

Jb = 1.4e-3; %<-Aluminium %2.0e-3 <-Quanser %Beam moment of inertia

Bb = 3.4e-3; %Beam viscous friction coefficient %to be estimated

k = 0.83; %Joint stiffness %to be estimated


%other important parameters

Beq = 1.0675e-06;  % 1.2195e-06; %same estimate of prev labs

Jeq = mot.J + Jh/gbox.N^2;
%Jeq = 6.7903e-7;

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

