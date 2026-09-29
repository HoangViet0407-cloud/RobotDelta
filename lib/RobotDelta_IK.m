function [theta, ok] = RobotDelta_IK(xyz, G)
%#codegen
%ROBOTDELTA_IK  Dong hoc nguoc robot delta.
%   [theta, ok] = RobotDelta_IK(xyz, G)
%   xyz   : vi tri TCP (tam ban dong) trong he toa do robot [m]
%           (goc = tam de, cao do truc dong co, z huong len)
%   G     : hinh hoc (params/RobotDelta_Geometry.m cho mo phong,
%           Greal trong params/RobotDelta_Real.m cho robot that)
%   theta : 3x1 goc canh tay tren so voi phuong ngang [deg], duong = cui xuong
%   ok    : 3x1 true neu chan voi toi duoc (false -> goc bi bao hoa tai bien)
%
%   Moi chan duoc tinh rieng theo so do thuc (CAD khong doi xung hoan toan):
%   diem giua 2 bi o dau canh tay A(th) = F + La*(cos(th)*e1 - sin(th)*ez)
%   phai cach diem giua 2 bi tren ban dong (xyz + b) dung bang Lr.

theta = zeros(3,1);
ok = true(3,1);
p = reshape(xyz,1,3);
for i = 1:3
    w  = G.F(i,:) - (p + G.b(i,:));
    a  =  2*G.La(i)*dot(w, G.e1(i,:));
    bb = -2*G.La(i)*dot(w, G.ez(i,:));
    c  = G.Lr(i)^2 - dot(w,w) - G.La(i)^2;
    k  = c/hypot(a,bb);
    ok(i) = abs(k) <= 1;
    th = atan2(bb,a) + G.branch(i)*acos(max(-1, min(1, k)));
    theta(i) = mod(th*180/pi + 180, 360) - 180;
end
end
