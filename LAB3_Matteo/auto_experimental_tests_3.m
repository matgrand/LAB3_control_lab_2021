% script to automatic do the experimental assignments

clear all;
close all;


we_are_in_a_simulation = 1; %0 = WE ARE NOT IN A SIMULATION
stop_time = 8;
enjoyment_time = 5;


%3.3  PID
disp("A classic: The position PID controller... ")
disp("Crypto may take over the economy, but the PID will always get the job done")

ass2_2_PID;

disp("Enjoy these beautiful responses <3")
pause(enjoyment_time);


%3.4 STATE SPACE

disp("Another classsic: the nominal and robust State Space Controller")

ass2_3_SS;

disp("Enjoy these less beautiful responses <3")
pause(enjoyment_time);

%3.5 LQR

disp("Now we are getting fancy: these LQR SS controllers will blow your mind")
disp("Give em the right cost function and they'll do their job better than how Apple steals your money")

ass2_4_1_4_LQR;
ass2_4_5_8_LQR;

disp("Enjoy these much more beautiful responses <3")
pause(enjoyment_time);

disp("And finally what you all came for: The FREQUENCY DEPENDENT LQR STATE SPACE CONTROL")
disp("With these controller we are going to the mooooooooooon!!!!!!!!!!!")

ass2_4_9_12LQR;




























