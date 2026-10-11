function D = rd_identdyn()
%RD_IDENTDYN  Nhan dang tham so mo hinh dong luc hoc rut gon (lib/RobotDelta_Dyn.m) tu Simscape.
%   Chay 2 quy dao (duong tron, gap-dat) tren RobotDelta_Sim, lay mo-men khop,
%   giai binh phuong toi thieu tau = Y*phi -> ghi params/RobotDelta_DynParams.m.
%   Kiem chung tren quy dao khac (di qua diem, nhanh hon) chua dung de nhan dang.
%   Tren robot that: lam lai voi mo-men uoc luong tu dong driver / do quan tinh roto.

RobotDelta_Params;
cases = {1, 2.0; 2, 2.0};                       % {mode, chu ky}: du lieu nhan dang
val   = {2, 1.2};                               % kiem chung: gap-dat nhanh hon
YY = []; TT = [];
for c = 1:size(cases,1)
    [Y, tq] = collect(Traj, cases{c,:}, G);
    YY = [YY; Y]; TT = [TT; tq];                %#ok<AGROW>
end
% chieu mo-men Simscape (theo q) so voi theta: tau_theta = s*tau_q (s = +-1)
sgn = G.s(1);
phi = YY \ (sgn*TT);
D.phi = phi;
fprintf('\n===== THAM SO DONG LUC HOC (nhan dang tu Simscape) =====\n');
fprintf('  Ib = %.5f kg*m^2   mr = %.4f kg*m   mp = %.4f kg\n', phi);
rep('nhan dang', YY, sgn*TT, phi);
[Yv, tv, t] = collect(Traj, val{:}, G);
e = rep(sprintf('kiem chung (gap-dat %.1f s)', val{2}), Yv, sgn*tv, phi);

f = figure('Visible', ternary(usejava('desktop'),'on','off'), 'Position',[60 60 1100 600]);
for i = 1:3
    subplot(3,1,i); plot(t, sgn*tv(i:3:end), 'k', t, Yv(i:3:end,:)*phi, 'r--'); grid on;
    ylabel(sprintf('\\tau_%d [N m]', i));
    if i == 1, title(sprintf('Kiem chung: Simscape (den) vs mo hinh rut gon (do), sai so RMS %.1f %%', e)); legend('Simscape','RobotDelta\_Dyn'); end
end
xlabel('t [s]');
L = rd_local(); exportgraphics(f, fullfile(char(L.dataDir), 'identdyn_kiemchung.png'));
if ~usejava('desktop'), close(f); end

fn = fullfile(rd_root(), 'params', 'RobotDelta_DynParams.m');
fid = fopen(fn, 'w');
fprintf(fid, '%%%% RobotDelta_DynParams.m -- TU SINH boi tools/rd_identdyn.m (%s). KHONG sua tay.\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf(fid, '%% Tham so mo hinh rut gon lib/RobotDelta_Dyn.m, tai truc canh tay. Sai so kiem chung RMS %.1f %%.\n', e);
fprintf(fid, 'Dyn.phi = [%.6g; %.6g; %.6g];   %% [Ib kg*m^2; mr kg*m; mp kg]\n', phi);
fclose(fid);
fprintf('  Da ghi %s\n', fn);
end

function [Y, tq, t] = collect(Traj, mode, period, G)
Tj = Traj; Tj.mode = mode; Tj.period = period;
S = exp_simulate(Tj, Tj.t_ramp + 2*period);
k = S.t > 0.05;                                  % bo qua luc khoi dong cua solver
t = S.t(k); tq = S.torque(k,:);
Y = zeros(3*numel(t), 3);
for j = 1:numel(t)
    [th, dth, ddth] = RobotDelta_IKd(t(j), Tj, G);
    [~, Yj] = RobotDelta_Dyn(th, dth, ddth, G, struct('phi',zeros(3,1)));
    Y(3*j-2:3*j, :) = Yj;
end
tq = reshape(tq', [], 1);
end

function e = rep(name, Y, tq, phi)
r = tq - Y*phi;
e = 100*rms(r)/rms(tq);
fprintf('  %-28s RMS tau %.3f N*m, sai so %.4f N*m (%.1f %%), max %.3f N*m\n', name, rms(tq), rms(r), e, max(abs(r)));
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
