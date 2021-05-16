%script to run a simulation both in normal mode and in real time

% REQUIRES simulink_system TO RUN THE SIMULATION

if exist('we_are_in_a_simulation','var') == 1
else
    disp("can't find we_are_in_a_simulation... assuming we_are_in_a_simulation = 1")
    we_are_in_a_simulation = 1;
end

if exist('stop_time','var') == 1
else
    disp("can't find stop_time... assuming stop_time = 5 s")
    stop_time = 5;
end

str_stop_time = string(stop_time);


%simulation
if (we_are_in_a_simulation == 1) 
    set_param(simulink_system,'StopTime',str_stop_time);
    save_system(simulink_system);
    sim(simulink_system);
    
%experiment
else
    open_system(simulink_system);
    set_param(simulink_system,'SimulationMode','external')
    set_param(simulink_system,'StopTime',str_stop_time);
    save_system(simulink_system);
    set_param(simulink_system,'SimulationCommand','connect');
    set_param(simulink_system,'SimulationCommand','start');   
   
    %pause(1);
    
    %w8 for simulation to finish
    while (not(strcmp('stopped',get_param(simulink_system,'SimulationStatus'))))
        pause(0.1);
    end
    
    close_system(simulink_system);
    
end

tmp.thh_scope = thh_scope1;
tmp.thd_scope = thd_scope1;
tmp.i_scope = i_scope1;