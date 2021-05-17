clear;
loadResonantParEst;
t0 = 0.2;
t1 = 0.7;
wc = 2*pi*50;
delta = 1/sqrt(2);

s = tf('s');
H1 = wc^2*s/(s^2+2*delta*wc*s+wc^2);

A = [0,0,1,0;...
     0,0,0,1;...
     -mld.k/gbox.N1^2/Jeq,mld.k/gbox.N1^2/Jeq,-(Beq+mot.Kt*mot.ke/Req)/Jeq,0;...
     mld.k/mld.Jb,-mld.k/mld.Jb,0,-mld.Bb/mld.Jb];
B = [0;...
     0;...
     mot.Kt*drv.dcgain/gbox.N1/Jeq/Req;...
     0];
Bd = [0;...
     0;...
     -1/gbox.N1^2/Jeq;...
     0];
C = [1,0,0,0];
T = [1,0,0,0;...
     1,1,0,0;...
     0,0,1,0;...
     0,0,1,1];
Tinv = inv(T);

A1 = Tinv*A*T;
B1 = Tinv*B;
Bd1 = Tinv*Bd;
C1 = C*T;
sysG = ss(A1,B1,C1,0);
[wn,zeta] = damp(sysG);
w0 = wn(3)*sqrt(1-2*zeta(3)^2);
% thetaH = 0.279*5/180*pi;%OTTIMO 0.188
% thetaD = 1*pi/36;
% u = sqrt(10);
% qI = 0.1;
% q11 = (1/thetaH)^2;
% q22 = (1/thetaD)^2;
% q33 = 6;
% q44 = 1;

thetaH = 0.2685*5/180*pi;%OTTIMO 0.187
thetaD = 0.585*pi/36;
u = sqrt(10);
qI = 0.1;
q11 = (1/thetaH)^2;
q22 = (1/thetaD)^2;
q33 = 6.6;
q44 = 0;

% thetaH = 0.2*5/180*pi;
% 
% thetaD = 0.4*pi/36;
% 
% u = sqrt(2);
r = 1/u^2;
% 
% qI = 0.1;
% q11 = (1/thetaH)^2;
% q22 = (1/thetaD)^2;
% q33 = 1;
% q44 = 0;

Aq1 = [0,1;...
       -w0^2,0];
Bq1 = [0;...
       1];
Cq1 = [sqrt(q22)*w0^2,0];
Dq1 = 0;
sigmaQ1 = ss(Aq1,Bq1,Cq1,Dq1);

Aq = Aq1;
Bq = [zeros(2,1),Bq1,zeros(2,2)];
Cq = [zeros(1,2);...
      Cq1;...
      zeros(2,2)];
Dq = diag([1/thetaH,Dq1,sqrt(q33),sqrt(q44)]);
sigmaQ = ss(Aq,Bq,Cq,Dq);

Aa = [A1,zeros(size(A1,1),size(Aq,2));...
      Bq,Aq];
Ba = [B1;...
      zeros(size(Aq,1),size(B1,2))];
Ca = [C1,zeros(1,2)];
Da = 0;
sigmaA = ss(Aa,Ba,Ca,Da);
Qa = [Dq'*Dq,Dq'*Cq;...
      Cq'*Dq,Cq'*Cq];
Ra = r;
Ae = [0,Ca;zeros(size(Aa,1),1),Aa];
Be = [0;Ba];
Ce = [0,Ca];

Qe = blkdiag(qI,Qa);
Re = Ra;
Na = zeros(size(Qa,1),size(Ra,2));

Ke = lqr(Ae,Be,Qe,Re)
bb = zeros(size(Aa,1),1);
bb = [bb;1];
xx = [Aa,Ba;Ca,Da]\bb;
Nxa = xx(1:end-1);
%Nx = reshape(Nx,[],1);
Nu = xx(end);
%Nu = reshape(Nu,[],1);
