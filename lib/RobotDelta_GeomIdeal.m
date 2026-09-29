function G = RobotDelta_GeomIdeal(D, Gref)
%ROBOTDELTA_GEOMIDEAL  Hinh hoc delta doi xung ly tuong tu kich thuoc do tren robot that.
%   G = RobotDelta_GeomIdeal(D, Gref)
%   D.R, D.r, D.La, D.Lr [m]  (xem params/RobotDelta_Real.m)
%   D.phi    [deg] 1x3 huong ra ngoai cua 3 canh tay
%   D.h_tool [m]   khoang cach tu mat phang tam bi xuong dau cong cu
%   D.z_home [m]   cao do vi tri nghi
%   Gref: G cua mo phong (giu nhanh nghiem IK va cac truong khac).

G = Gref;
for i = 1:3
    u = [cosd(D.phi(i)) sind(D.phi(i)) 0];
    G.F(i,:)  = D.R*u;
    G.e1(i,:) = u;
    G.ez(i,:) = [0 0 1];
    G.b(i,:)  = D.r*u + [0 0 D.h_tool];
end
G.La   = D.La*[1 1 1];
G.Lr   = D.Lr*[1 1 1];
G.home = [0 0 D.z_home];
end
