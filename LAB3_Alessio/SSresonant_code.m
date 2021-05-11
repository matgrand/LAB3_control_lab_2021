clear all;
load_params_resonantLoad;
SS_LRQ_commonStart;

% design K with eigenvalue placement
p0 = -wn*exp(1i*phi);
p1 = -wn*exp(1i*phi/2);
p = 3*[p0, conj(p0), p1, conj(p1)];
K = place(A, B, p);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% integral action with exo-system for step reference
Ae = [0 C; zeros(4,1) A];
Be = [0; B];

p5 = -wn;

p_rob = 3*[p0, conj(p0), p1, conj(p1), p5];

Ke = acker(Ae, Be, p_rob);
Ki_robust = Ke(1);
K_robust = [Ke(2) Ke(3) Ke(4) Ke(5)];