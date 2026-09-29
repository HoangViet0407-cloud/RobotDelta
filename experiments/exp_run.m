function R = exp_run(testName, backend, varargin)
%EXP_RUN  Chay 1 thi nghiem: cau hinh -> quy dao khop -> kiem tra gioi han -> chay -> luu -> ve.
%   R = exp_run('E2_DuongTron')                      % 'dry': chi tao + kiem tra quy dao
%   R = exp_run('E2_DuongTron', 'sim')               % chay tren mo phong
%   R = exp_run('E2_DuongTron', 'real')              % chay tren robot that (RobotLink + firmware)
%   R = exp_run('E2_DuongTron', 'sim', 'period', 1)  % tham so rieng cua thi nghiem (xem tests/)
%   Them 'force', true de van gui xuong robot that khi quy dao vi pham gioi han (KHONG nen).
%
%   Ket qua: <thu muc du lieu>/<yyyyMMdd_HHmmss>_<ten>_<backend>_<nguoi dung>/
%     (thu muc du lieu mac dinh <du an>/data, doi trong local/RobotDelta_Local.m)
%     run.mat        toan bo cau hinh + du lieu (R), ca thong tin may / nguoi chay
%     lenh_khop.csv  goc dat cho 3 dong co theo thoi gian (gui cho firmware)
%     do_tay.csv     (E1) bang de dien toa do do bang tay
%     *.png, bao_cao.txt

if nargin < 2, backend = 'dry'; end
backend = validatestring(backend, {'dry','sim','real'});
force = false;
iF = find(cellfun(@(x) (ischar(x) || isstring(x)) && strcmpi(x,'force'), varargin), 1);
if ~isempty(iF), force = varargin{iF+1}; varargin(iF:iF+1) = []; end

RobotDelta_Params;                      % G, Traj, Motor, Lim
RobotDelta_Real;                        % Real, Greal
cfg = feval(testName, Traj, varargin{:});
if strcmp(backend,'real'), Gk = Greal; else, Gk = G; end

fprintf('\n===== %s | %s | %s =====\n%s\n', testName, backend, char(datetime('now')), cfg.mo_ta);
J = exp_jointtraj(cfg.Traj, cfg.T, Gk, Lim, Motor);
if isfield(cfg,'io'), J.io = cfg.io(J.t); else, J.io = zeros(size(J.t)); end
if ~J.pass && strcmp(backend,'real') && ~force
    error('Quy dao vi pham gioi han -> KHONG gui xuong robot: %s', strjoin(J.issues,'; '));
end

R = struct('test',testName, 'backend',backend, 'thoi_gian',datetime('now'), 'cfg',cfg, ...
           'J',J, 'G',Gk, 'Lim',Lim, 'Motor',Motor);
L = rd_local();
R.meta = struct('nguoi_dung', L.user, 'may', L.may, 'matlab', version('-release'), 'du_an', rd_root());
R.folder = newRunFolder(L, [testName '_' backend]);
writetable(array2table([J.t J.theta J.io], 'VariableNames', ...
    {'t_s','theta1_deg','theta2_deg','theta3_deg','io'}), fullfile(R.folder,'lenh_khop.csv'));

switch backend
    case 'sim'
        R.sim = exp_simulate(cfg.Traj, cfg.T);
    case 'real'
        link = RobotLink(Real.port, Real.baud);
        cleanup = onCleanup(@() delete(link));
        link.home();
        R.real = link.run(J.t, J.theta, J.io);
end

if isfield(cfg,'diem_do')               % bang do tay (thuoc / anh chup) cho thi nghiem do chinh xac
    d = cfg.diem_do; n = numel(d.id);
    tb = table(d.id, d.xyz(:,1)*1e3, d.xyz(:,2)*1e3, d.xyz(:,3)*1e3, nan(n,1), nan(n,1), nan(n,1), ...
        'VariableNames', {'id','x_dat_mm','y_dat_mm','z_dat_mm','x_do_mm','y_do_mm','z_do_mm'});
    writetable(tb, fullfile(R.folder,'do_tay.csv'));
end
save(fullfile(R.folder,'run.mat'), 'R');
exp_analyze(R);
fprintf('Ket qua luu tai: %s\n', R.folder);
end

function f = newRunFolder(L, name)
% ten thu muc: thoi gian + thi nghiem + nguoi chay -> nhieu nguoi ghi chung 1 thu muc khong trung
user = regexprep(char(L.user), '[^A-Za-z0-9]', '_');
f = fullfile(char(L.dataDir), [char(datetime('now','Format','yyyyMMdd_HHmmss')) '_' name '_' user]);
mkdir(f);
end
