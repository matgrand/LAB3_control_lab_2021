% Sampling time (ADC, DAC)
Ts = 0.001; 

%high-pass filter "real derivative" to get angular velocities
H_wc = 2*pi*50;
H_damp = 1/sqrt(2);
H_num = [H_wc^2, 0];
H_den = [1, 2*H_damp*H_wc, H_wc^2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% state vector [thh, thd, wh, wd]
% state space nominal tracking
a32 = mld.k/(gbox.N^2 * Jeq);
a33 = -(Beq + (mot.Kt*mot.Ke)/Req)/Jeq;
a42 = -mld.k/mld.Jb -mld.k/(gbox.N^2 * Jeq);
a44 = -mld.Bb/mld.Jb;
a43 = a44 - a33;
dimA = 4;
A = [0, 0, 1, 0; ...
    0, 0, 0, 1; ...
    0, a32, a33, 0; ...
    0, a42, a43, a44];
    
b3 = mot.Kt*drv.dcgain/(gbox.N * Jeq *Req);
B = [0; 0; b3; -b3];
C = [1, 0, 0, 0];
D = 0;

Nxu = [A, B; C, 0]\[zeros(dimA,1);1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4)];
Nu = Nxu(dimA+1);

% design parameters
t_sett_5 = 0.85;
Mp = 0.3;

dump = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
wn = 3/(dump * t_sett_5);
phi = atan(sqrt(1-dump^2)/dump);