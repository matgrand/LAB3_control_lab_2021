% assignment 3.2 (1)
%estimation of the elastic joint parameters


clear all;
close all;

we_are_testing = 1; %if 1 use simulation, if 0 actual experiment

load_param_resonance_load;
important_parameters;

if (we_are_testing == 1) 
    simulink_system = "model3_2_TEST";
else
    simulink_system = "exp3_2_elastic_joint_estimation";
end

stop_time = 2; 

disp("move the beam in 1 second!!!!");

run_simulation;
[t, ref, thh, thd, i, out1] = plot_and_save(ans, "exp3_2_elastic_joint_estimation", 0)

abs_thd = abs(thd);

figure;
plot(t, abs_thd);

grid on

figure
[pks, locs] = findpeaks(abs_thd);
scatter(locs, pks);
f = fit(locs, pks, 'exp1');
sigma = f.b*1000;
T_avg = mean(diff(locs));
w_est = (pi/T_avg)*1000;

delta_est = (w_est-sqrt(w_est^2+4*sigma^2))/(2*sigma)
wn_est = w_est/sqrt(1-delta_est^2)
Bb_est = Jb*(2*delta_est*wn_est)
k_est = Jb*wn_est^2


%umbe's version

TF = islocalmax(abs_thd, 'MinSeparation', 100, 'MinProminence', 0.5);
t1 = [0; t(TF)];
thd_abs1 = [10; abs_thd(TF)];
M = numel(t1);
Phi = -(0:1:M-1);
Phi = [Phi', ones(size(Phi',1),1)];
Y = log(thd_abs1);
thetaLs = inv(Phi'*Phi)*Phi'*Y;
Xi = thetaLs(1);
dampingFactorEst = Xi/sqrt(pi^2+Xi^2)
TkEst = diff(t1);
wkEst = pi*TkEst.^-1;
wEst = mean(wkEst)
wnEst = wEst/sqrt(1-dampingFactorEst^2) 
















