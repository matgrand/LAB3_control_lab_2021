%this script plots the data saved from experiments
%CREATES t_saved ref_saved thh_saved thd_saved i_saved par_saved

%REQUIRES 'out' (load a .mat file before running this)

t_saved = out.t;
ref_saved = out.ref;
par_saved = out.par;
thh_saved = out.thh;
th_saved = out.thd;
i_saved = out.i;

figure;
grid on;
subplot(1,2,1);
plot(out.t,out.thh);
hold on;
plot(out.t, out.ref);
if exist('simulink_system','var') == 1
    title(simulink_system);
end

subplot(1,2,2);
plot(out.t,out.thd);
title("thd")

