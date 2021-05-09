function [t, ref, thh, thd, i, out] = plot_and_save(simulink_out, str_name, additional_data)
    %PLOT_AND_SAVE_SCOPES accepts the variable "out"("ans") from simulink, a
    %string for the name and additional variables to be saved (in a struct)(0 for no additional vars)
    
    O = simulink_out;
    
    t = O.thh_scope.time;
    ref = O.thh_scope.signals(1).values;
    thh = O.thh_scope.signals(2).values;
    thd = O.thd_scope.signals(1).values;
    i = O.i_scope.signals(1).values;
    
    %plotting
    figure;
    title(str_name);
    grid on;
    subplot(1,2,1);
    plot(t,thh);
    hold on;
    plot(t, ref);
    title(str_name,'Interpreter','none');

    subplot(1,2,2);
    plot(t,thd);
    title("thd")
    
    %saving
    filename = strcat("saved_data/", str_name, '.mat');
    out.t = t;
    out.ref = ref;
    out.thh = thh;
    out.thd = thd;
    out.i = i;
    out.par = additional_data;
    
    save(filename, 'out');  
end

