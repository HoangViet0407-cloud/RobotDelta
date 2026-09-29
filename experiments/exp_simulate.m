function S = exp_simulate(Traj, T)
%EXP_SIMULATE  Chay RobotDelta_Sim voi quy dao Traj trong T giay (khong mo Mechanics Explorer).
%   S.t [s], S.tcp_dat, S.tcp [m], S.theta_dat, S.theta [deg], S.torque [N*m]
%   Traj truyen qua SimulationInput nen KHONG bi RobotDelta_Params (InitFcn) ghi de.

in = Simulink.SimulationInput('RobotDelta_Sim');
in = in.setVariable('Traj', Traj);
in = in.setModelParameter('StopTime', num2str(T), 'SimMechanicsOpenEditorOnUpdate', 'off');
out = sim(in);
S.t         = out.tcp.Time;
S.tcp       = nx3(out.tcp.Data);
S.theta     = nx3(out.theta_do.Data);
S.torque    = nx3(out.momen.Data);
S.tcp_dat   = onto(out.tcp_dat, S.t);
S.theta_dat = onto(out.theta_dat, S.t);
end

function d = nx3(d)
if ndims(d) == 3, d = squeeze(d)'; end
end

function y = onto(ts, t)
[tt, iu] = unique(ts.Time);
d = nx3(ts.Data);
y = interp1(tt, d(iu,:), t);
end
