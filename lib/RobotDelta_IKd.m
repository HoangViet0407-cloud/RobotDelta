function [th, dth, ddth] = RobotDelta_IKd(t, Traj, G)
%#codegen
%ROBOTDELTA_IKD  Goc tay va dao ham chinh xac tren quy dao: theta [deg], dtheta [deg/s], ddtheta [deg/s^2].
%   [th, dth, ddth] = RobotDelta_IKd(t, Traj, G)
%   Sai phan trung tam cua IK tren quy dao tai t-h, t, t+h (h = 1e-4 s):
%     dth  = (th(t+h) - th(t-h)) / 2h           ddth = (th(t+h) - 2 th(t) + th(t-h)) / h^2
%   Quy dao da duoc lam muot (van toc, gia toc lien tuc) nen hai dao ham nay on dinh.
%   Dung cho khoi SPS che do "Provide signal and derivatives" -> khong tre nhu bo loc.

h = 1e-4;
th  = RobotDelta_IK(RobotDelta_Traj(t,     Traj), G);
thp = RobotDelta_IK(RobotDelta_Traj(t + h, Traj), G);
thm = RobotDelta_IK(RobotDelta_Traj(t - h, Traj), G);
dth  = (thp - thm)/(2*h);
ddth = (thp - 2*th + thm)/h^2;
end
