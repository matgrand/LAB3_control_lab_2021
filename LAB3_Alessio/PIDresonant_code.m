% Sampling time (ADC, DAC, BB)
Ts = 0.001; 

% beam mechanical dynamics
% P is from u to theta hub
s = tf('s');
D1_tau = Jeq*mld.Jb*s^3 + (Jeq*mld.Bb+mld.Jb*Beq)*s^2 +(Beq*mld.Bb+mld.k*(Jeq+(mld.Jb)/gbox.N1^2))*s +mld.k*(Beq+(mld.Bb)/gbox.N1^2);
P = drv.dcgain*mot.Kt*(mld.Jb*s^2+mld.Bb*s+mld.k)/gbox.N1/s/(Req*D1_tau+mot.Kt*mot.Ke*(mld.Jb*s^2+mld.Bb*s+mld.k));

% design parameters
t_sett_5 = 0.85;
Mp = 0.3;
alpha = 4;
beta = 10;
Kw = 5/t_sett_5;

% PID formulas
delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
w_gc_star = 3/(delta * t_sett_5);
phi_m_star = atan(2*delta/(sqrt(sqrt(1+4*delta^4)-2*delta^2)));

[mag, phase, xxx] = bode(P, w_gc_star); %phase in degrees
delta_K = 1/mag;
delta_phi = - pi + phi_m_star - phase/180*pi;
Kp = delta_K * cos(delta_phi);

Td = (tan(delta_phi)+sqrt(tan(delta_phi)^2 + 4/alpha))/(2*w_gc_star);
Ti = alpha*Td;
Kd = Kp*Td;
Ki = Kp/Ti;
Tl = 1/(beta*w_gc_star);         