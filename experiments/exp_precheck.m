function exp_precheck()
%EXP_PRECHECK  Kiem tra truoc khi thuc nghiem tren robot that (khong can phan cung).
%   1. Hinh hoc CAD: do doi xung, hinh binh hanh; so voi mo hinh doi xung (params/RobotDelta_Real.m)
%   2. Vung lam viec an toan: voi toi, goc tay, nghieng bi mat trau, goc tay - thanh truyen
%   3. Do phan giai TCP theo 1 vi buoc dong co
%   4. Toc do, xung buoc, mo-men can cho cac quy dao mau (chay mo phong)
%   Ket qua in ra man hinh va luu vao <thu muc du lieu>/<thoi gian>_precheck_<nguoi dung>/

RobotDelta_Params;                      % G, Traj, Motor, Lim
RobotDelta_Real;                        % Real, Greal
L = rd_local();
out = fullfile(char(L.dataDir), [char(datetime('now','Format','yyyyMMdd_HHmmss')) '_precheck_' ...
    regexprep(char(L.user), '[^A-Za-z0-9]', '_')]);
mkdir(out);
diary(fullfile(out,'bao_cao.txt')); stopDiary = onCleanup(@() diary('off'));
vis = 'on'; if ~usejava('desktop'), vis = 'off'; end
warn = {};

%% 1. Hinh hoc
fprintf('\n===== 1. HINH HOC TU CAD (he toa do robot, mm) =====\n');
d   = cross(G.e1, G.ez, 2);
R   = sum(G.F .* G.e1, 2)';             % tam de -> truc dong co
off = sum(G.F .* d, 2)';                % mat phang canh tay lech khoi tam
r   = sum(G.b .* G.e1, 2)';
phi = atan2d(G.e1(:,2), G.e1(:,1))';
rows = {'R  tam de -> truc dong co', R; 'mat phang tay lech tam', off; 'r  tam ban -> cap bi', r; ...
        'La canh tay tren', G.La; 'Lr thanh truyen (tam-tam)', G.Lr; ...
        'khoang cach 2 bi tren tay', G.wArm; 'khoang cach 2 bi tren ban', G.wPlat};
fprintf('  %-28s %8s %8s %8s\n', '', 'Chan 1', 'Chan 2', 'Chan 3');
for k = 1:size(rows,1), fprintf('  %-28s %8.1f %8.1f %8.1f\n', rows{k,1}, rows{k,2}*1e3); end
fprintf('  %-28s %8.1f %8.1f %8.1f   [deg]\n', 'huong canh tay phi', phi);
if max(R) - min(R) > 1e-3 && ~Real.dung_CAD
    warn{end+1} = sprintf(['CAD: 3 truc dong co KHONG cach deu tam (R = %s mm) nhung robot that khai bao doi xung ' ...
        '(Real.dung_CAD = false) -> kiem tra lai so do trong RobotDelta_Real.m'], mat2str(round(R*1e3,1)));
end
fprintf('  Robot that: %s\n', ternary(Real.dung_CAD, 'dung hinh hoc CAD (da do, giong CAD)', 'dung mo hinh doi xung tu RobotDelta_Real.m'));
fprintf('  Thanh truyen: khoi luong rieng %g kg/m^3\n', VatLieu.rho_thanh);
fprintf('  File CAD them tay:\n');
if ~rd_checkgeom()
    warn{end+1} = 'File CAD trong cad/solidworks KHAC phien ban voi cum lap SolidWorks -> chi tiet ve sai vi tri (xem canh bao o muc 1)';
end
if any(abs(G.wArm - G.wPlat) > 0.5e-3)
    warn{end+1} = sprintf(['CAD: 2 bi tren tay cach %.0f mm nhung tren ban dong %.0f mm -> 2 thanh truyen ' ...
        'khong song song, ban dong bi nghieng. Nen bang nhau.'], G.wArm(1)*1e3, G.wPlat(1)*1e3);
end

%% 2. Vung lam viec
fprintf('\n===== 2. VUNG LAM VIEC AN TOAN (gioi han trong RobotDelta_Params.m) =====\n');
fprintf('  goc tay %s deg | nghieng bi mat trau <= %g deg | goc tay-thanh %s deg\n', ...
    mat2str(Lim.theta), Lim.tilt_rodend, mat2str(Lim.gamma));
lbl = {'an toan','ngoai tam voi','goc tay','nghieng bi mat trau','gan ky di'};
xs = -0.16:0.004:0.16; [X, Y] = meshgrid(xs, xs);
zs = G.home(3) + [0.05 0 -0.05];
fig = figure('Visible',vis,'Position',[50 50 1450 430]);
tl = tiledlayout(fig, 1, numel(zs), 'TileSpacing','compact');
for iz = 1:numel(zs)
    code = zeros(size(X)); dth = nan(size(X));
    for k = 1:numel(X)
        p = [X(k) Y(k) zs(iz)];
        [th, ok] = RobotDelta_IK(p, G);
        if ~all(ok), code(k) = 1; continue; end
        if any(th < Lim.theta(1) | th > Lim.theta(2)), code(k) = 2; continue; end
        M = RobotDelta_Metrics(p, th', G);
        if any(M.tilt > Lim.tilt_rodend), code(k) = 3; continue; end
        if any(M.gamma < Lim.gamma(1) | M.gamma > Lim.gamma(2)), code(k) = 4; continue; end
        dth(k) = max(abs(th - RobotDelta_IK(p, Greal)));
    end
    rho = hypot(X, Y); bad = code > 0;
    [rs, ib] = min(rho(bad)); cb = code(bad); why = lbl{cb(ib)+1};
    fprintf('  z = %6.1f mm: vong tron an toan quanh tam R = %3.0f mm (bi chan boi: %s)', zs(iz)*1e3, rs*1e3, why);
    if ~Real.dung_CAD, fprintf('; CAD vs robot that lech goc toi %.1f deg', max(dth(~bad))); end
    fprintf('\n');
    res.z(iz) = zs(iz); res.r(iz) = rs;
    nexttile(tl);
    imagesc(xs*1e3, xs*1e3, code, [0 4]); axis xy equal tight; hold on;
    plot(rs*1e3*cos(0:0.05:2*pi+0.05), rs*1e3*sin(0:0.05:2*pi+0.05), 'w-', 'LineWidth', 1.5);
    title(sprintf('z = %.0f mm  (R an toan %.0f mm)', zs(iz)*1e3, rs*1e3)); xlabel('x [mm]'); ylabel('y [mm]');
end
colormap([0.30 0.69 0.31; 0.60 0.60 0.60; 1.00 0.60 0.00; 0.89 0.10 0.11; 0.50 0.30 0.70]);
cb = colorbar('Ticks', 0.4 + (0:4)*0.8, 'TickLabels', lbl); cb.Layout.Tile = 'east';
exportgraphics(fig, fullfile(out,'vung_lam_viec.png'), 'Resolution', 110);
if strcmp(vis,'off'), close(fig); end
[rmin, izmin] = min(res.r);
if rmin < 0.06
    warn{end+1} = sprintf('Vung an toan hep: tai z = %.0f mm chi con R = %.0f mm quanh tam', res.z(izmin)*1e3, rmin*1e3);
end

%% 3. Do phan giai
fprintf('\n===== 3. DO PHAN GIAI (tai home) =====\n');
stepDeg = 360/(Motor.steps_rev*Motor.microstep*Motor.gear);
th0 = RobotDelta_IK(G.home, G); Jac = zeros(3);
for i = 1:3
    e = zeros(3,1); e(i) = 1e-3;
    Jac(:,i) = (RobotDelta_FK(th0+e, G) - RobotDelta_FK(th0-e, G))/2e-3;   % m/deg
end
res1 = vecnorm(Jac)*stepDeg*1e3;
fprintf('  1 vi buoc = %.4f deg (%d buoc x %d vi buoc x ti so %g)\n', stepDeg, Motor.steps_rev, Motor.microstep, Motor.gear);
fprintf('  TCP dich chuyen / 1 vi buoc:  %.3f  %.3f  %.3f mm  (dong co 1 2 3)\n', res1);
fprintf('  TCP dich chuyen / 1 buoc du:  %.2f  %.2f  %.2f mm\n', res1*Motor.microstep);
if isfield(Motor, 'encoder_ppr')
    encDeg = 360/(4*Motor.encoder_ppr*Motor.gear);
    fprintf('  Encoder: 1 dem = %.4f deg canh tay -> TCP %.4f  %.4f  %.4f mm\n', encDeg, vecnorm(Jac)*encDeg*1e3);
end

%% 4. Quy dao mau
fprintf('\n===== 4. QUY DAO MAU (IK + mo phong) =====\n');
cases = {'Dung yen tai home (tai trong tinh)', 0, 2; ...
         sprintf('Duong tron R%.0f mm, %.1f s/vong', Traj.radius*1e3, Traj.period), 1, Traj.t_ramp + 2*Traj.period; ...
         sprintf('Gap-dat, %.1f s/chu ky', Traj.period), 2, Traj.t_ramp + 2*Traj.period};
for c = 1:size(cases,1)
    Tj = Traj; Tj.mode = cases{c,2};
    fprintf('\n  -- %s --\n', cases{c,1});
    J = exp_jointtraj(Tj, cases{c,3}, G, Lim, Motor);
    S = exp_simulate(Tj, cases{c,3});
    tq = max(abs(S.torque), [], 1);
    fprintf('  mo-men can (mo phong) max:    %.2f  %.2f  %.2f N*m  (tai canh tay)\n', tq);
    if Motor.gear ~= 1
        fprintf('  quy ve truc dong co (/%g):    %.3f %.3f %.3f N*m  (chua tinh hieu suat hop so)\n', Motor.gear, tq/Motor.gear);
    end
    if ~J.pass, warn{end+1} = sprintf('Quy dao "%s": %s', cases{c,1}, strjoin(J.issues,'; ')); end %#ok<AGROW>
    if isnan(Lim.torque)
        if c == size(cases,1)
            warn{end+1} = 'Chua khai bao Lim.torque / Lim.omega: so mo-men o tren voi datasheet dong co + driver'; %#ok<AGROW>
        end
    elseif any(tq > Lim.torque)
        warn{end+1} = sprintf('Quy dao "%s" can %.2f N*m > Lim.torque = %g N*m', cases{c,1}, max(tq), Lim.torque); %#ok<AGROW>
    end
end

%% Tong ket
fprintf('\n===== CANH BAO / VIEC CAN LAM =====\n');
if isempty(warn), fprintf('  Khong co.\n'); end
for k = 1:numel(warn), fprintf('  %d. %s\n', k, warn{k}); end
fprintf('\nDa luu: %s\n', out);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
