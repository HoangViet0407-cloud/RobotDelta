function exp_analyze(R)
%EXP_ANALYZE  Ve & danh gia 1 lan chay.
%   exp_analyze(R)                          % R tu exp_run
%   exp_analyze('<thu muc ket qua>')        % chay lai, vd sau khi da dien do_tay.csv
%       duong dan day du, hoac chi ten thu muc (tim trong thu muc du lieu cua may / <du an>/data)
%   Luu: quy_dao.png, khop.png, kiem_tra.png, bao_cao.txt vao thu muc ket qua.

if ~isstruct(R)
    folder = findRunFolder(char(R));
    S = load(fullfile(folder,'run.mat')); R = S.R;
    R.folder = folder;          % du lieu co the do may khac tao -> dung vi tri hien tai
end
J = R.J; Lim = R.Lim; f = R.folder;
vis = 'on'; if ~usejava('desktop'), vis = 'off'; end
rep = {sprintf('%s | %s | %s', R.test, R.backend, char(R.thoi_gian)), R.cfg.mo_ta, ''};
rep{end+1} = sprintf('Quy dao khop: %s', ternary(J.pass, 'DAT gioi han', ['VI PHAM - ' strjoin(J.issues,'; ')]));
rep{end+1} = sprintf('  toc do max %s deg/s | xung buoc max %s Hz | nghieng bi max %s deg', ...
    mat2str(round(J.summary.omega_max)), mat2str(round(J.summary.pulse_max)), mat2str(round(J.summary.tilt_max,1)));

% --- du lieu thuc te (neu co) ---
hasSim  = isfield(R,'sim');
hasReal = isfield(R,'real') && ~isempty(R.real);
if hasSim
    e = vecnorm(R.sim.tcp - R.sim.tcp_dat, 2, 2)*1e3;
    rep{end+1} = sprintf('Mo phong: sai so TCP max %.2f mm, RMS %.2f mm', max(e), rms(e));
    rep{end+1} = sprintf('Mo phong: mo-men dong co max %s N*m', mat2str(round(max(abs(R.sim.torque),[],1),3)));
    if ~isnan(Lim.torque) && any(max(abs(R.sim.torque),[],1) > Lim.torque)
        rep{end+1} = sprintf('  CANH BAO: vuot mo-men cho phep %g N*m', Lim.torque);
    end
end
if hasReal
    L = R.real; tr = (L.t_ms - L.t_ms(1))/1e3;
    thr = [L.theta1 L.theta2 L.theta3];
    xr = zeros(numel(tr),3);
    for k = 1:numel(tr), xr(k,:) = RobotDelta_FK(thr(k,:)', R.G)'; end
    thc = interp1(J.t, J.theta, min(tr, J.t(end)));
    ej = max(abs(thr - thc), [], 1);
    rep{end+1} = sprintf('Robot that: lech goc (do - dat) max %s deg (co tre do bo dem)', mat2str(round(ej,2)));
end

% --- Hinh 1: quy dao TCP ---
fig = figure('Visible',vis,'Name','quy_dao','Position',[50 50 1100 450]);
subplot(1,2,1); hold on; grid on; axis equal; view(35,25);
plot3(J.xyz(:,1)*1e3, J.xyz(:,2)*1e3, J.xyz(:,3)*1e3, 'k--', 'DisplayName','dat');
if hasSim,  plot3(R.sim.tcp(:,1)*1e3, R.sim.tcp(:,2)*1e3, R.sim.tcp(:,3)*1e3, 'b', 'DisplayName','mo phong'); end
if hasReal, plot3(xr(:,1)*1e3, xr(:,2)*1e3, xr(:,3)*1e3, 'r', 'DisplayName','that (FK goc do)'); end
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]'); legend('Location','best'); title('Quy dao TCP');
subplot(1,2,2); hold on; grid on;
if hasSim, plot(R.sim.t, vecnorm(R.sim.tcp - R.sim.tcp_dat, 2, 2)*1e3, 'b', 'DisplayName','mo phong'); end
if hasReal
    xc = zeros(numel(tr),3);
    for k = 1:numel(tr), xc(k,:) = RobotDelta_Traj(min(tr(k), J.t(end)), R.cfg.Traj)'; end
    plot(tr, vecnorm(xr - xc, 2, 2)*1e3, 'r', 'DisplayName','that');
end
xlabel('t [s]'); ylabel('|sai so TCP| [mm]'); title('Sai so bam quy dao');
if hasSim || hasReal, legend('Location','best'); end
saveFig(fig, f);

% --- Hinh 2: goc & toc do khop ---
fig = figure('Visible',vis,'Name','khop','Position',[80 80 1100 600]);
c = lines(3);
subplot(2,1,1); hold on; grid on;
for i = 1:3
    plot(J.t, J.theta(:,i), '--', 'Color', c(i,:), 'DisplayName', sprintf('dat %d', i));
    if hasSim,  plot(R.sim.t, R.sim.theta(:,i), 'Color', c(i,:), 'DisplayName', sprintf('mo phong %d', i)); end
    if hasReal, plot(tr, thr(:,i), ':', 'Color', c(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('that %d', i)); end
end
yline(Lim.theta, 'r-', 'HandleVisibility','off');
ylabel('goc tay [deg]'); legend('Location','eastoutside'); title('Goc canh tay (duong = cui xuong)');
subplot(2,1,2); hold on; grid on;
plot(J.t, J.omega); ylabel('toc do [deg/s]'); xlabel('t [s]');
if ~isnan(Lim.omega), yline([-1 1]*Lim.omega, 'r-'); end
saveFig(fig, f);

% --- Hinh 3: cac gioi han ---
fig = figure('Visible',vis,'Name','kiem_tra','Position',[110 110 1100 650]);
leg = {'chan 1','chan 2','chan 3'};
subplot(3,1,1); plot(J.t, J.tilt); grid on; yline(Lim.tilt_rodend, 'r-');
ylabel('nghieng bi [deg]'); title('Goc nghieng bi mat trau (gioi han do)'); legend(leg, 'Location','eastoutside');
subplot(3,1,2); plot(J.t, J.gamma); grid on; yline(Lim.gamma, 'r-');
ylabel('tay-thanh [deg]'); title('Goc giua canh tay va thanh truyen'); legend(leg, 'Location','eastoutside');
subplot(3,1,3); grid on; hold on;
if hasSim
    plot(R.sim.t, R.sim.torque);
    if ~isnan(Lim.torque), yline([-1 1]*Lim.torque, 'r-'); end
    title('Mo-men dong co (mo phong)');
else
    plot(J.t, nan(numel(J.t),3));
    title('Mo-men dong co: chay backend ''sim'' de co');
end
legend(leg, 'Location','eastoutside'); ylabel('[N*m]'); xlabel('t [s]');
saveFig(fig, f);

% --- Do chinh xac tu bang do tay (E1) ---
fn = fullfile(f,'do_tay.csv');
if isfile(fn)
    tb = readtable(fn);
    if any(~isnan(tb.x_do_mm))
        [AP, RP, ids] = iso9283(tb);
        rep{end+1} = 'Do chinh xac (ISO 9283, tu do_tay.csv):';
        for j = 1:numel(ids)
            rep{end+1} = sprintf('  diem %d: AP = %.2f mm, RP = %.2f mm', ids(j), AP(j), RP(j)); %#ok<AGROW>
        end
    else
        rep{end+1} = 'do_tay.csv chua co so do -> dien cot *_do_mm roi chay lai exp_analyze(thu muc).';
    end
end

fprintf('%s\n', rep{:});
fid = fopen(fullfile(f,'bao_cao.txt'), 'w'); fprintf(fid, '%s\n', rep{:}); fclose(fid);
end

function f = findRunFolder(name)
L = rd_local();
cand = {name, fullfile(char(L.dataDir), name), fullfile(rd_root(), name), fullfile(rd_root(), 'data', name)};
k = find(cellfun(@(c) isfile(fullfile(c, 'run.mat')), cand), 1);
if isempty(k), error('Khong tim thay run.mat trong "%s"', name); end
d = dir(fullfile(cand{k}, 'run.mat'));
f = d.folder;                           % duong dan tuyet doi
end

function [AP, RP, ids] = iso9283(tb)
% AP: khoang cach tu trung binh cac lan do den diem dat; RP = trung binh + 3*do lech chuan
% cua khoang cach cac lan do toi trung binh. Chi dung cac truc co so do (vd. do tren giay: x, y).
ids = unique(tb.id); AP = nan(numel(ids),1); RP = AP;
Pm = [tb.x_do_mm tb.y_do_mm tb.z_do_mm]; Pc = [tb.x_dat_mm tb.y_dat_mm tb.z_dat_mm];
use = any(~isnan(Pm), 1);
for j = 1:numel(ids)
    r = tb.id == ids(j) & ~isnan(tb.x_do_mm);
    if ~any(r), continue; end
    m = Pm(r,use); mu = mean(m, 1);
    AP(j) = norm(mu - Pc(find(r,1),use));
    l = vecnorm(m - mu, 2, 2); RP(j) = mean(l) + 3*std(l);
end
end

function saveFig(fig, folder)
exportgraphics(fig, fullfile(folder, [fig.Name '.png']), 'Resolution', 110);
if strcmp(fig.Visible, 'off'), close(fig); end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
