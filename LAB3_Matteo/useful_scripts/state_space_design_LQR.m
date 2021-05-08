load_param_resonance_load;
important_parameters;

%STATE SPACE
s = tf('s');

%real derivative transfer function for finding and filtering w
w_c = 2*pi*50;
delta_f = 1/sqrt(2);
H_w = (w_c^2*s)/(s^2+2*delta_f*w_c*s+w_c^2); %real derivative
[numH_w,denH_w] = tfdata(H_w, 'v'); %to be used it in a simulink block

%for findiing poles (maybe useless)
delta = (log(1/Mp))/(sqrt(pi^2+(log(1/Mp))^2));
wn = 3/(delta*ts5);
phi = atan(sqrt(1-delta^2)/delta);

%state space design
% %matrix A 
% a31 = -k/(N*N*Jeq);
% a32 = -a31;
% a33 = -(1/Jeq)*(Beq+(Kt*Ke)/Req);
% a41 = k/Jb;
% a42 = -a41;
% a44 = -Bb/Jb;
% A = [0 0 1 0; 
%     0 0 0 1;
%     a31 a32 a33 0;
%     a41 a42 0 a44];
% 
% %B
% b3 = (Kt*Kdrv)/(N*Jeq*Req);
% B = [0;0; b3;0];


%let's use A' B' etc
a32 = mld.k/gbox.N^2/Jeq;
a33 = -(1/Jeq)*(Beq + mot.Kt*mot.Ke/Req);
a42 = -(mld.k/mld.Jb) - mld.k/Jeq/gbox.N^2;
a43 = -mld.Bb/mld.Jb + (1/Jeq)*(Beq + mot.Kt*mot.Ke/Req);
a44 = -mld.Bb/mld.Jb;

b31 = mot.Kt*drv.dcgain/gbox.N/Jeq/Req;
b41 = -b31;

A = [ 0, 0, 1, 0;
    0, 0, 0, 1;
    0, a32, a33, 0;
    0, a42, a43, a44 ];

B = [0; 0; b31; b41];

C = [1 0 0 0];
D = [0];

%Nx Nu
F = [A B; C D];
Nxu = F\[0; 0; 0; 0; 1];
Nx = [Nxu(1); Nxu(2); Nxu(3); Nxu(4)];
Nu = Nxu(5);