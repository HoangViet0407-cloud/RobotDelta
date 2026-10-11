function [tau, p, J] = rd_dyn_sym(q, dq, ddq, G, phi)
%RD_DYN_SYM  Dong luc hoc nguoc rut gon, viet cho CasADi (chay duoc ca voi so double).
%   [tau, p, J] = rd_dyn_sym(q, dq, ddq, G, phi)
%   q, dq, ddq : 3x1 goc tay [rad], [rad/s], [rad/s^2] (quy uoc RobotDelta_IK: duong = cui xuong)
%   phi        : [Ib; mr; mp] (params/RobotDelta_DynParams.m)
%   Giong lib/RobotDelta_Dyn.m (da kiem chung voi Simscape) nhung khong dung if / gan vao mang
%   double -> CasADi tao duoc bieu thuc va dao ham giai tich cho acados.

g = 9.81;
A = cell(1,3); A1 = A; A2 = A;
for i = 1:3
    e1 = G.e1(i,:)'; ez = G.ez(i,:)';
    A{i}  = G.F(i,:)' + G.La(i)*( cos(q(i))*e1 - sin(q(i))*ez) - G.b(i,:)';   % tam khuyu - b_i
    A1{i} =             G.La(i)*(-sin(q(i))*e1 - cos(q(i))*ez);
    A2{i} =            -G.La(i)*( cos(q(i))*e1 - sin(q(i))*ez);
end
% dong hoc thuan: giao 3 mat cau |p - A_i| = Lr
a2 = 2*(A{2} - A{1}); k2 = G.Lr(1)^2 - G.Lr(2)^2 + A{2}'*A{2} - A{1}'*A{1};
a3 = 2*(A{3} - A{1}); k3 = G.Lr(1)^2 - G.Lr(3)^2 + A{3}'*A{3} - A{1}'*A{1};
n  = cross(a2, a3);
p0 = (k2*cross(a3, n) + k3*cross(n, a2)) / (n'*n);
n  = -n * sign(n(3)) / sqrt(n'*n);               % chon nghiem phia duoi
v  = p0 - A{1};
B  = n'*v;
p  = p0 + (-B + sqrt(B^2 - (v'*v - G.Lr(1)^2)))*n;
% Jacobian va gia toc ban dong tu rang buoc s_i'*(dp - A1_i*dq_i) = 0
s  = [p - A{1}, p - A{2}, p - A{3}];
Dq = [s(:,1)'*A1{1}, 0, 0; 0, s(:,2)'*A1{2}, 0; 0, 0, s(:,3)'*A1{3}];
J  = s' \ Dq;
dp = J*dq;
r  = [0; 0; 0] * q(1);                            % cung kieu voi q (double hoac SX)
for i = 1:3
    sd = dp - A1{i}*dq(i);
    r(i) = s(:,i)'*(A2{i}*dq(i)^2 + A1{i}*ddq(i)) - sd'*sd;
end
ddp = s' \ r;
tau = phi(1)*ddq - g*phi(2)*cos(q) + J'*(phi(3)*(ddp - [0; 0; -g]));
end
