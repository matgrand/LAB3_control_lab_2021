function [t, ref, out] = plot_scope(scope, plot_string)
%PLOT_SCOPE Accepts a scope from simulink with 2 signals (ref =1 , out =1) (log type: structure with time)

t = scope.time;
ref = scope.signals(1).values;
out = scope.signals(2).values;

figure;
plot(t, out);
hold on;
grid on;
plot(t, ref);
title(plot_string);

end

