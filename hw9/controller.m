function out = controller(u,P)
 
% process inputs(40%)
xd    = u(1:3);
% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
 
xd_1dot = [0; 0; 0];
xd_2dot = [0; 0; 0];
 
% calculate errors, eq 17-18
ex = x - xd;
ev = v - xd_1dot;
 
% inertial frame 3-axis
e3 = [0; 0; 1];
 
% thrust magnitude control(15%)
A = -P.kx*ex - P.kv*ev - P.mass*P.gravity*e3 + P.mass*xd_2dot;
f = dot(-A, R*e3);
 
% desired R and omega(15%)
Rc = [1 0 0; 0 1 0; 0 0 1];
Omegac = [0; 0.1; 1];
 
% inertia matrix
J = diag([P.Jxx P.Jyy P.Jzz]);
 
% error(20%)
eR     = (1/2)*vee(Rc.'*R - R.'*Rc);
eOmega = Omega - R.'*Rc*Omegac;
 
% moment vector control(10%)
M = -P.kR*eR - P.kOmega*eOmega + cross(Omega, J*Omega);
 
% calculate SO(3) error function, Psi
Psi = (1/2)*trace(eye(3) - Rc.'*R);
 
out = [f;M];
end
