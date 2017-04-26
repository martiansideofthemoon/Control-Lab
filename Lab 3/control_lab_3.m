Mp = 0.027;
lp = 0.153;
r  = 0.0826;
g  = 9.810;
Jeq= 1.23e-4;
Jp = 1.1e-4;
Rm = 3.3;
Kt = 0.02797;
Km = 0.02797;

%what we set:
q1 = 1; 
q2 = 1;
q3 = 1;
q4 = 1;
R  = 1;

den  = Jp*Jeq + Mp*lp^2*Jeq + Jp*Mp*r^2; 

A = [0              0                             1                  0;
     0              0                             0                  1;
     0      r*Mp^2*lp^2*g/den         -Kt*Km*(Jp + Mp*lp^2)/(den*Rm) 0;
     0 Mp*lp*g*(Jeq + Mp*r^2)/den       -Kt*Km*Mp*lp*r/(den*Rm)      0];

B =[0 0 Kt*(Jp + Mp*lp^2)/(den*Rm) Kt*Mp*lp*r/(den*Rm)]';

C = eye(4);

D = zeros(4,1);

sys_ss = ss(A,B,C,D);

diagonal = [q1 q2 q3 q4];

Q = diag(diagonal);

K = lqr(A,B,Q,R);