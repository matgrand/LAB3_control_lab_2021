function [Kp, Ki, Kd, Tl, Kw, num_C, den_C] = PID_designer_CT(plant_sys, settling_time, overshoot, wgc_multiplier, alpha)
    %function that calculates the PID gains, the anti-windupgain
    %and real derivative time Tl, given settling time and overshoot
    
ts_5 = settling_time;
Mp = overshoot;
sysP = plant_sys;

psi = log(1/Mp)/sqrt(pi^2+(log(1/Mp))^2);
wgc_star = 3/(psi*ts_5);
Tl = 1/(wgc_multiplier*wgc_star);
phim_star = atan((2*psi)/sqrt(sqrt(1+4*psi^4)-2*psi^2));
[magP_star, phaseP_star_deg] = bode(sysP, wgc_star);
phaseP_star = phaseP_star_deg*0.0174532925199433;

% Parameters for the controller design
delta_K = magP_star^(-1);
delta_phi = -pi+phim_star-phaseP_star;
Td = (tan(delta_phi)+sqrt((tan(delta_phi))^2+4/alpha))/(2*wgc_star);
Ti = alpha*Td;

%PID GAINS
Kp = delta_K*cos(delta_phi);
Ki = Kp/Ti;
Kd = Kp*Td;
Tw = ts_5/5;
Kw = 1/Tw; % anti wind-up

%PID Tf
% Controller PID
s = tf('s');
sysC_rd = Kp+Ki/s+Kd*s/(Tl*s+1);
[num_C, den_C] = tfdata(sysC_rd, 'v');

end