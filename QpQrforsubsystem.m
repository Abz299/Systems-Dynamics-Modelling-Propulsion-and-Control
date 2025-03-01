syms Kt lh Kv Ixx Iyy Izz Lp Mq mg l_theta l_h l_boom K_tau K_v omega_coll V x6 Ir omega_rotor gamma theta_rest KD

A = [
    0,                 1,                0,                   0,                     0,                          0;
    (-mg*l_theta)/Ixx,   -Lp/Ixx,          0,      0,       0,           (-Kv*lh)/Ixx;
    0,                 0,                0,                   1,                     0,                          0;
    0, 0, (-mg*l_theta*cos(theta_rest))/Iyy,   (-Mq)/Iyy,       0,   0;
    0,                 0,                0,                   0,                     0,                          1;
    (90*K_tau*l_boom*V)/Izz,                          0,                   0, 0,                    0, 0
];

B = [
    0,                 0;
    (130*Kt*lh*6)/(Ixx*780),     0
    0,                 0;
    0, (90*Kt*l_boom*6)/(Iyy*540);
    0,                 0;
    0,                 0
];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;]; 

D = [0, 0]; 

f = [0, 0, 1, 0, 0, 0];

% Convert state space to transfer function (symbolically)
syms s
I = eye(size(A)); % Identity matrix
Qp_s = simplify(C * inv(s*I - A) * B); 

disp('T.F Qp(s) =');
disp(Qp_s);

e = [1, 0, 0, 0, 0, 0];

syms s
I = eye(size(A)); % Identity matrix
Qr_s = simplify(e * inv(s*I - A) * B); 

disp('T.F Qr(s) =');
disp(Qr_s);
