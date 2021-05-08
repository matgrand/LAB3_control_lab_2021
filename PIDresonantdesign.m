% Approximated plant 
Ts = 0.001;
Req = mot.R + sens.curr.Rs;

Jeq = mot.J + mld.Jh/gbox.N^2; % 1st test
% %Beq = mot.B + mld.B/gbox.N^2;
% Beq = 0; % 1st test

BeqEst = 1.0675e-06;
% JeqEst = 6.7903e-07;
tauSfEst = 0.0125;

Beq = BeqEst; % 2nd test
% Jeq = JeqEst; % 2nd test

Bb = mld.Bb;
k = mld.k;

num = (mot.Kt*drv.dcgain)*[mld.Jb, Bb, k];
D_tau = [Jeq*mld.Jb, Jeq*Bb+mld.Jb*Beq, Beq*Bb+k*(Jeq+mld.Jb/gbox.N^2), k*(Beq+Bb/gbox.N^2), 0];
den = gbox.N*Req*D_tau+gbox.N*mot.Kt*mot.Ke*[0, mld.Jb, Bb, k, 0];
sysP = tf(num, den);
den_B = [mld.Jb, Bb];

% Useful results
ts_5 = 0.85;
Mp = 0.3;
psi = log(1/Mp)/sqrt(pi^2+(log(1/Mp))^2);
wgc_star = 3/(psi*ts_5);
phim_star = atan((2*psi)/sqrt(sqrt(1+4*psi^4)-2*psi^2));
[magP_star, phaseP_star_deg] = bode(sysP, wgc_star);
phaseP_star = phaseP_star_deg*deg2rad;

% Parameters for the controller design
delta_K = magP_star^(-1);
delta_phi = -pi+phim_star-phaseP_star;
alpha = 4;
Td = (tan(delta_phi)+sqrt((tan(delta_phi))^2+4/alpha))/(2*wgc_star);
Ti = alpha*Td;

% PID parameters
Kp = delta_K*cos(delta_phi)
Ki = Kp/Ti
Kd = Kp*Td
Tw = ts_5/5;
Kw = 1/Tw % anti wind-up