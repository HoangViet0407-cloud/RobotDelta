function build_RobotDelta_NMPC()
%BUILD_ROBOTDELTA_NMPC  Tao models/RobotDelta_NMPC.slx: robot Simscape dieu khien bang NMPC (acados).
%   Sao chep RobotDelta_Sim.slx, thay khoi IK (truyen thang) bang subsystem NMPC:
%     ChuanBi   (roi rac, Ts) : do goc tay -> trang thai x0, tham chieu N buoc tu quy dao (IK)
%     acados    (S-function)  : giai OCP SQP-RTI -> u0 = gia toc dat, x1
%     NoiSuy    (lien tuc)    : lenh cho driver trong 1 chu ky: q = qc + dqc*tau + u*tau^2/2
%   Can chay truoc: RobotDelta_NMPC_build(true)  (tao work/acados_solver_sfunction_robotdelta)
%   Chay lai script nay moi khi build_RobotDelta_Sim tao lai RobotDelta_Sim.slx.

src = 'RobotDelta_Sim'; m = 'RobotDelta_NMPC';
sfun = 'acados_solver_sfunction_robotdelta';
assert(exist(sfun, 'file') == 3 || isfile(fullfile(rd_root(), 'work', [sfun '.' mexext])), ...
    'Chua co S-function: chay RobotDelta_NMPC_build(true) truoc');
if bdIsLoaded(m), close_system(m, 0); end
load_system(src);
fn = fullfile(rd_root(), 'models', [m '.slx']);
save_system(src, fn); close_system(src, 0);        % src van giu nguyen tren dia
load_system(m);

% ---- go khoi IK, nho noi den cua 3 cong ra ----
ik = [m '/IK'];
pos = get_param(ik, 'Position');
ph = get_param(ik, 'PortHandles');
dst = cell(1,3);
for k = 1:3
    l = get_param(ph.Outport(k), 'Line');
    dst{k} = get_param(l, 'DstPortHandle');
    delete_line(l);
end
delete_line(get_param(ph.Inport(1), 'Line'));
delete_block(ik);

% ---- subsystem NMPC ----
s = [m '/NMPC'];
add_block('simulink/Ports & Subsystems/Subsystem', s, 'Position', pos + [0 0 40 40]);
Simulink.SubSystem.deleteContents(s);
blk = @(lib, name, p, varargin) add_block(lib, [s '/' name], 'Position', p, varargin{:});
blk('simulink/Sources/In1', 't',        [20 40 50 54]);
blk('simulink/Sources/In1', 'theta_do', [20 120 50 134], 'Port', '2');
blk('simulink/Signal Routing/Mux', 'Mux_do_lenh', [70 105 75 165], 'Inputs', '2');
% goc do + goc lenh qua CUNG 1 khoi Memory -> sai so bam lay dung cung thoi diem (Memory tranh vong dai so)
blk('simulink/Discrete/Memory', 'Mem', [95 120 125 150], ...
    'InitialCondition', 'repmat(RobotDelta_IK(RobotDelta_Traj(0, Traj), G), 2, 1)');
[~, ch] = addMatlabFcn([s '/ChuanBi'], [160 30 330 190], chuanBiCode(), {'G','Traj','Nmpc'});
ch.ChartUpdate = 'DISCRETE'; ch.SampleTime = 'Nmpc.Ts';
hS = blk('simulink/User-Defined Functions/S-Function', 'acados', [400 30 540 190], 'FunctionName', sfun);
set_param(hS, 'Name', 'acados_NMPC');
blk('simulink/Discrete/Unit Delay', 'd_truoc', [400 300 440 330], 'InitialCondition', 'zeros(3,1)', 'SampleTime', 'Nmpc.Ts');
blk('simulink/Discrete/Unit Delay', 'x1_truoc', [400 240 440 270], 'InitialCondition', '[RobotDelta_IK(RobotDelta_Traj(0, Traj), G)*pi/180; 0; 0; 0]', 'SampleTime', 'Nmpc.Ts');
addMatlabFcn([s '/NoiSuy'], [620 40 760 160], noiSuyCode(), {});
outN = {'theta_dat', 'dtheta_dat', 'ddtheta_dat'};
for k = 1:3
    blk('simulink/Sinks/Out1', outN{k}, [820 50+40*(k-1) 850 64+40*(k-1)], 'Port', num2str(k));
end
blk('simulink/Sinks/To Workspace', 'log_trangthai', [620 210 700 230], 'VariableName', 'nmpc_trangthai', 'SaveFormat', 'Timeseries');
blk('simulink/Sinks/To Workspace', 'log_tgian',     [620 250 700 270], 'VariableName', 'nmpc_tgian',     'SaveFormat', 'Timeseries');
L = @(a, b) add_line(s, a, b, 'autorouting', 'on');
L('t/1', 'ChuanBi/1'); L('theta_do/1', 'Mux_do_lenh/1'); L('NoiSuy/1', 'Mux_do_lenh/2'); L('Mux_do_lenh/1', 'Mem/1');
L('Mem/1', 'ChuanBi/2'); L('x1_truoc/1', 'ChuanBi/3'); L('d_truoc/1', 'ChuanBi/4'); L('ChuanBi/7', 'd_truoc/1');
L('ChuanBi/1', 'acados_NMPC/1'); L('ChuanBi/1', 'acados_NMPC/2');            % lbx_0 = ubx_0 = x0
L('ChuanBi/2', 'acados_NMPC/3'); L('ChuanBi/3', 'acados_NMPC/4'); L('ChuanBi/4', 'acados_NMPC/5');
L('acados_NMPC/3', 'x1_truoc/1');
L('t/1', 'NoiSuy/1'); L('ChuanBi/5', 'NoiSuy/2'); L('ChuanBi/6', 'NoiSuy/3'); L('acados_NMPC/1', 'NoiSuy/4');
L('acados_NMPC/2', 'log_trangthai/1'); L('acados_NMPC/4', 'log_tgian/1');
L('NoiSuy/1', 'theta_dat/1'); L('NoiSuy/2', 'dtheta_dat/1'); L('NoiSuy/3', 'ddtheta_dat/1');

% ---- noi lai tang tren ----
sp = get_param(s, 'PortHandles');
add_line(m, 'Clock/1', 'NMPC/1', 'autorouting', 'on');
add_line(m, 'Robot_Delta/1', 'NMPC/2', 'autorouting', 'on');
for k = 1:3
    for d = dst{k}(:)', add_line(m, sp.Outport(k), d, 'autorouting', 'on'); end
end
set_param(get_param(sp.Outport(1), 'Line'), 'Name', 'theta_dat');
set_param(m, 'InitFcn', 'RobotDelta_Params; addpath(fullfile(rd_root(), ''work''));');
set_param(s, 'BackgroundColor', 'lightBlue');
save_system(m);
close_system(m);
fprintf('==> Da tao %s\n', fn);
end

function c = chuanBiCode()
c = strjoin({ ...
'function [x0, yref0, yref, yrefe, tk, xc, d] = ChuanBi(t, q_do_lenh, x1_truoc, d_truoc, G, Traj, Nmpc)'
'% Moi chu ky Ts: trang thai cho acados + tham chieu N buoc.'
'%   xc = trang thai LENH tai t (lenh xung da phat, noi tiep chu ky truoc -> lenh lien tuc)'
'%   d  = sai so bam do duoc = goc encoder - goc lenh (cung thoi diem)  -> phan hoi'
'%   x0 = xc + [d; 0]: goc thuc te uoc luong. Driver tre / mat buoc -> d ~= 0 -> NMPC lap lai quy dao'
'%   tham chieu tu IK cua quy dao: [q; dq; ddq] tai t, t+Ts, ..., t+N*Ts'
'd2r = pi/180; N = Nmpc.N; Ts = Nmpc.Ts;'
'd  = (q_do_lenh(1:3) - q_do_lenh(4:6))*d2r;'
'xc = x1_truoc(:) - [d_truoc(:); 0; 0; 0];  % x1 du bao tu x0 = xc + d -> bo d de con lai lenh'
'x0 = xc + [d; 0; 0; 0];'
'tk = t;'
'Y = zeros(9, N);'
'for j = 0:N-1'
'    [th, dth, ddth] = RobotDelta_IKd(t + j*Ts, Traj, G);'
'    Y(:, j+1) = [th; dth; ddth]*d2r;'
'end'
'[th, dth] = RobotDelta_IKd(t + N*Ts, Traj, G);'
'yref0 = Y(:, 1);'
'yref  = reshape(Y(:, 2:N), [], 1);'
'yrefe = [th; dth]*d2r;'
'end'}, newline);
end

function c = noiSuyCode()
c = strjoin({ ...
'function [theta_dat, dtheta_dat, ddtheta_dat] = NoiSuy(t, tk, xc, u)'
'% Lenh lien tuc cho driver trong chu ky [tk, tk+Ts): gia toc u (tu NMPC) khong doi'
'% -> goc / van toc / gia toc dat [deg, deg/s, deg/s^2] cho 3 cong cua khoi SPS.'
'% Tren robot that: vi dieu khien phat xung STEP theo dung bieu thuc nay.'
'tau = t - tk;'
'r2d = 180/pi;'
'theta_dat   = (xc(1:3) + xc(4:6)*tau + u(:)*tau^2/2)*r2d;'
'dtheta_dat  = (xc(4:6) + u(:)*tau)*r2d;'
'ddtheta_dat = u(:)*r2d;'
'end'}, newline);
end

function [h, ch] = addMatlabFcn(path, pos, code, param)
h = add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
ch = sfroot().find('-isa','Stateflow.EMChart','Path',path);
ch.Script = code;
for p = cellstr(param)                 % tham so lay tu workspace, co dinh (kich thuoc mang theo Nmpc.N)
    if isempty(p{1}), continue; end
    d = ch.find('-isa','Stateflow.Data','Name',p{1}); d.Scope = 'Parameter'; d.Tunable = false;
end
end
