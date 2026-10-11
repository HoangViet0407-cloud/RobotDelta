function [R, B] = nmpc_simscape(TrajIn, T, varargin)
%NMPC_SIMSCAPE  Chay RobotDelta_NMPC (NMPC + robot Simscape day du), so voi IK truyen thang (RobotDelta_Sim).
%   [R, B] = nmpc_simscape()                    quy dao trong RobotDelta_Params
%   [R, B] = nmpc_simscape(Traj, T, 'tau_max', 0.8)   (sau RobotDelta_NMPC_build(true, 0.8))
%   R: ket qua NMPC, B: ket qua IK truyen thang (khong gioi han mo-men). Mo-men lay tu Simscape
%   (mo hinh day du), khong phai mo hinh rut gon cua NMPC -> kiem chung sai lech mo hinh.
%   'tau_max' chi de hien thi: gioi han thuc nam trong S-function da build.

RobotDelta_Params;
if nargin >= 1 && ~isempty(TrajIn), Tj = TrajIn; else, Tj = Traj; end
if nargin < 2 || isempty(T), T = Tj.t_ramp + 2*Tj.period; end
o = inputParser; o.addParameter('ve', true); o.addParameter('tau_max', Nmpc.tau_max); o.parse(varargin{:}); o = o.Results;

addpath(fullfile(rd_root(), 'work'));
in = Simulink.SimulationInput('RobotDelta_NMPC');
in = in.setVariable('Traj', Tj);
in = in.setModelParameter('StopTime', num2str(T), 'SimMechanicsOpenEditorOnUpdate', 'off');
t0 = tic; out = sim(in); R.t_mophong = toc(t0);
R = pack(R, out);
R.trangthai = squeeze(out.nmpc_trangthai.Data);
R.tgian = squeeze(out.nmpc_tgian.Data);
B = exp_simulate(Tj, T);
B.e_tcp = vecnorm(B.tcp - B.tcp_dat, 2, 2);

fprintf('\n===== NMPC tren robot Simscape (%s, T = %g s) =====\n', mo(Tj), T);
fprintf('  %-22s %12s %12s %14s\n', '', 'TCP RMS [mm]', 'TCP max [mm]', 'max|tau| [N m]');
fprintf('  %-22s %12.3f %12.3f %14.3f\n', 'IK truyen thang', rms(B.e_tcp)*1e3, max(B.e_tcp)*1e3, max(abs(B.torque(:))));
fprintf('  %-22s %12.3f %12.3f %14.3f   (gioi han %.2f)\n', 'NMPC', rms(R.e_tcp)*1e3, max(R.e_tcp)*1e3, max(abs(R.torque(:))), o.tau_max);
fprintf('  acados: thoi gian giai TB %.3f ms, max %.3f ms | %d/%d chu ky loi (status ~= 0)\n', ...
    mean(R.tgian)*1e3, max(R.tgian)*1e3, nnz(R.trangthai ~= 0), numel(R.trangthai));

if o.ve
    f = figure('Name', 'NMPC Simscape', 'Position', [60 60 1000 700]);
    subplot(3,1,1); plot(B.t, B.theta_dat(:,1), 'k--', B.t, B.theta(:,1), 'b', R.t, R.theta(:,1), 'r'); grid on;
    ylabel('\theta_1 [deg]'); legend('quy dao (IK)', 'IK truyen thang', 'NMPC', 'Location', 'best');
    title(sprintf('Robot Simscape: %s', mo(Tj)));
    subplot(3,1,2); plot(B.t, max(abs(B.torque),[],2), 'b', R.t, max(abs(R.torque),[],2), 'r');
    yline(o.tau_max, 'r:'); grid on; ylabel('max|\tau| Simscape [N m]');
    subplot(3,1,3); plot(B.t, B.e_tcp*1e3, 'b', R.t, R.e_tcp*1e3, 'r'); grid on; ylabel('sai so TCP [mm]'); xlabel('t [s]');
    L = rd_local(); exportgraphics(f, fullfile(char(L.dataDir), 'nmpc_simscape.png'));
end
end

function R = pack(R, out)
nx3 = @(d) reshape(permute(d, [3 1 2]), size(d,3), []);
g = @(ts) iff(ndims(ts.Data) == 3, nx3(ts.Data), ts.Data);
R.t = out.tcp.Time; R.tcp = g(out.tcp); R.theta = g(out.theta_do); R.torque = g(out.momen);
[tt, iu] = unique(out.tcp_dat.Time); d = g(out.tcp_dat);
R.tcp_dat = interp1(tt, d(iu,:), R.t);
R.e_tcp = vecnorm(R.tcp - R.tcp_dat, 2, 2);
end

function v = iff(c, a, b), if c, v = a; else, v = b; end, end
function s = mo(Tj)
n = {'dung yen', 'duong tron', 'gap-dat', 'di qua diem'};
s = sprintf('%s, chu ky %.1f s', n{Tj.mode+1}, Tj.period);
end
