function [xyz, ok] = RobotDelta_FK(theta, G)
%#codegen
%ROBOTDELTA_FK  Dong hoc thuan: goc canh tay [deg] -> vi tri TCP [m] (he toa do robot).
%   [xyz, ok] = RobotDelta_FK(theta, G)
%   Dung de tinh TCP tu goc do duoc tren robot that (encoder / dem buoc).
%   ok = false neu 3 thanh truyen khong the gap nhau (bo goc khong hop le).

C = zeros(3,3);
for i = 1:3
    th = theta(i)*pi/180;
    C(i,:) = G.F(i,:) + G.La(i)*(cos(th)*G.e1(i,:) - sin(th)*G.ez(i,:)) - G.b(i,:);
end
% giao 3 mat cau |p - C_i| = Lr_i: tru tung cap -> 2 mat phang -> 1 duong thang
a2 = 2*(C(2,:) - C(1,:));  k2 = G.Lr(1)^2 - G.Lr(2)^2 + dot(C(2,:),C(2,:)) - dot(C(1,:),C(1,:));
a3 = 2*(C(3,:) - C(1,:));  k3 = G.Lr(1)^2 - G.Lr(3)^2 + dot(C(3,:),C(3,:)) - dot(C(1,:),C(1,:));
n  = cross(a2, a3);
p0 = (k2*cross(a3,n) + k3*cross(n,a2)) / dot(n,n);
n  = n/norm(n);
if n(3) > 0, n = -n; end                 % chon nghiem phia duoi (ban dong nam duoi de)
v  = p0 - C(1,:);
B  = dot(n,v);
D  = B^2 - (dot(v,v) - G.Lr(1)^2);
ok = D >= 0;
xyz = (p0 + (-B + sqrt(max(D,0)))*n).';
end
