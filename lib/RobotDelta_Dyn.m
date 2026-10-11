function [tau, Y, p, dp, ddp] = RobotDelta_Dyn(th, dth, ddth, G, D)
%#codegen
%ROBOTDELTA_DYN  Dong luc hoc nguoc rut gon (mo hinh du bao cho NMPC).
%   [tau, Y] = RobotDelta_Dyn(th, dth, ddth, G, D)
%   th, dth, ddth : 3x1 goc tay [deg], [deg/s], [deg/s^2] (quy uoc RobotDelta_IK: duong = cui xuong)
%   D             : tham so (params/RobotDelta_DynParams.m): D.phi = [Ib; mr; mp]
%   tau           : 3x1 mo-men tai truc canh tay [N*m], cung chieu theta
%   Y             : 3x3 ma tran hoi quy, tau = Y*phi (tuyen tinh theo tham so -> nhan dang bang LSQ)
%
%   Mo hinh (Codourey): moi thanh truyen chia 2 dau -> khuyu tay va ban dong.
%     Ib = quan tinh canh tay quanh truc + m_khuyu*La^2         [kg*m^2]
%     mr = m_tay*r_trongtam + m_khuyu*La                         [kg*m]
%     mp = ban dong + tai + phan thanh truyen dau duoi           [kg]
%   tau = Ib*ddq - g*mr*cos(q) + J'*mp*(ddp - gvec),  dp = J*dq
%   J lay tu rang buoc |p + b_i - A_i(q_i)| = Lr_i, khong can tinh nguoc thanh truyen.

g  = 9.81; gv = [0; 0; -g];
q  = th(:)*pi/180; dq = dth(:)*pi/180; ddq = ddth(:)*pi/180;
p  = RobotDelta_FK(th, G);
S = zeros(3); Dq = zeros(3); rhs = zeros(3,1);
A1 = zeros(3); A2 = zeros(3);
for i = 1:3
    c = cos(q(i)); s = sin(q(i));
    A    = G.F(i,:)' + G.La(i)*( c*G.e1(i,:)' - s*G.ez(i,:)');
    A1(:,i) =        G.La(i)*(-s*G.e1(i,:)' - c*G.ez(i,:)');      % dA/dq
    A2(:,i) =       -G.La(i)*( c*G.e1(i,:)' - s*G.ez(i,:)');      % d2A/dq2
    si = p + G.b(i,:)' - A;
    S(i,:) = si'; Dq(i,i) = si'*A1(:,i);
end
J  = S \ Dq;                                   % dp = J*dq
dp = J*dq;
for i = 1:3
    sd = dp - A1(:,i)*dq(i);                   % dao ham cua s_i
    rhs(i) = S(i,:)*(A2(:,i)*dq(i)^2 + A1(:,i)*ddq(i)) - sd'*sd;
end
ddp = S \ rhs;
Y   = [ddq, -g*cos(q), J'*(ddp - gv)];
tau = Y*D.phi(:);
end
