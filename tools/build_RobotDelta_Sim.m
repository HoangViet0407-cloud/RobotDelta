function build_RobotDelta_Sim()
%BUILD_ROBOTDELTA_SIM  Tao RobotDelta_Sim.slx (gon, chay duoc) tu RobotDelta.slx (import SolidWorks).
%   File goc RobotDelta.slx KHONG bi sua. Chay lai script nay moi khi import lai CAD.
%
%   Cac buoc:
%   1. Go 3 moi han sai Part1 <-> Ga_DinhMat (lam 3 khop dong co bi khoa cung)
%   2. Thay 12 moi han bi <-> o bi mat trau bang Spherical Joint
%   3. Khoa cung cac khop du (mat bich, truc ren, bi, dau thanh truyen) dung goc CAD,
%      xoa cac khop tao vong kin du
%   4. Sua trong luc ve -Z (import de sai la -Y)
%   5. Do hinh hoc bang KinematicsSolver -> params/RobotDelta_Geometry.m (dung cho IK)
%      He toa do robot: goc = tam de, cao do truc dong co, z huong len
%   6. Dong co: goc vao (Input motion) + do goc, mo-men; them khung TCP + cam bien
%   7. Gom subsystem: De | Chan_1..3 | Ban_Dong -> Robot_Delta; them Quy_Dao, IK, Scope

rd_addpath();
root = rd_root();                                         % thu muc du an (khong phu thuoc may)
mdlDir = fullfile(root,'models');
src = 'RobotDelta'; m = 'RobotDelta_Sim';
W_ARM = 0.068;   % [m] khoang cach 2 bi tren canh tay = tren ban dong -> 2 thanh truyen song song (CAD: 66 mm)
if bdIsLoaded(m), close_system(m,0); end
if bdIsLoaded(src), close_system(src,0); end
evalin('base','RobotDelta_DataFile;');

load_system(src);
save_system(src, fullfile(mdlDir,[m '.slx']));   % model dang mo gio ten la m
set_param(m,'PreLoadFcn','','InitFcn','');
% bo moi duong dan tuyet doi (file CAD tim theo MATLAB path) -> model chay tren moi may
rd_fixpaths(m);
% smiData lay tu base workspace (RobotDelta_Params) -> Model Workspace khong can file
ws = get_param(m,'ModelWorkspace'); ws.DataSource = 'Model File'; ws.clear;

%% 1. go moi han de <-> canh tay
for k = 1:3
    ph = get_param([m '/Part1_' num2str(k) '_RIGID'],'PortHandles');
    for p = [ph.LConn ph.RConn]
        l = get_param(p,'Line');
        if l>0 && any(strcmp(peerNames(l,p),'Ga_DinhMat_1_RIGID')), delete_line(l); end
    end
end

%% 2. gia tri khop tai tu the CAD
set_param(m,'SimscapeLogType','all','StopTime','0');
out = sim(m); lg = out.get(get_param(m,'SimscapeLogName'));
q0 = @(j,prim,f) first(lg.(j).(prim).(f).series.values(unitOf(f)));
qMot = [q0('Revolute15','Rz','q') q0('Revolute16','Rz','q') q0('Revolute17','Rz','q')];

%% 3. khoa / xoa khop du
lockRev = {'Revolute6','Revolute22','Revolute23','Revolute20','Revolute21','Revolute4','Revolute18','Revolute19', ...
           'Revolute','Revolute1','Revolute2','Revolute3','Revolute7','Revolute8', ...
           'Revolute9','Revolute10','Revolute11','Revolute12','Revolute13','Revolute14'};
for i=1:numel(lockRev), j=lockRev{i}; lockJoint(m,j,q0(j,'Rz','q'),[0 0 0]); end
for j = {'Cylindrical1','Cylindrical6'}, lockJoint(m,j{1},q0(j{1},'Rz','q'),[0 0 q0(j{1},'Pz','p')]); end
lockJoint(m,'Planar',q0('Planar','Rz','q'),[q0('Planar','Px','p') q0('Planar','Py','p') 0]);
for j = {'Cylindrical3','Cylindrical4','Cylindrical5','Cylindrical','Revolute5','Cylindrical2'}
    joinPeers(m,j{1}); delete_block([m '/' j{1}]);
end

%% 4. khop cau (bi mat trau)
arm  = {'bi_mattrauM6_V2_11','Bimattrau_M6_V2_15'; 'bi_mattrauM6_V2_12','Bimattrau_M6_V2_16';
        'bi_mattrauM6_V2_10','Bimattrau_M6_V2_14'; 'bi_mattrauM6_V2_6', 'Bimattrau_M6_V2_10';
        'bi_mattrauM6_V2_2', 'Bimattrau_M6_V2_1';  'bi_mattrauM6_V2_3', 'Bimattrau_M6_V2_2'};
plat = {'Bimattrau_M6_V2_4','Bimattrau_M6_V2_9','Bimattrau_M6_V2_5','Bimattrau_M6_V2_6','Bimattrau_M6_V2_8','Bimattrau_M6_V2_7'};
for i=1:6, weld2sph(m,[arm{i,1} '_RIGID'],[arm{i,2} '_RIGID'],sprintf('Cau_Tay_%d',i)); end
for i=1:6, weld2sph(m,'Assem1_1_RIGID',[plat{i} '_RIGID'],sprintf('Cau_Ban_%d',i)); end

%% 5. trong luc
set_param([m '/MechanismConfiguration'],'GravityVector','[0 0 -9.80665]');
save_system(m);

%% 5b. hinh binh hanh: doi 2 bi tren truc ren cua moi canh tay cho cach nhau W_ARM
adjustArmSpacing(m, qMot, W_ARM);
save_system(m);

%% 6. do hinh hoc -> RobotDelta_Geometry.m
G = measureGeometry(mdlDir, m, qMot);
writeGeometryFile(fullfile(root,'params','RobotDelta_Geometry.m'), G);
evalin('base','RobotDelta_Params;');

%% 6b. vat lieu thanh truyen: khoi luong + quan tinh tinh tu hinh hoc voi VatLieu.rho_thanh
rods = find_system(m,'LookUnderMasks','all','RegExp','on','ExtGeomFileName','^ThanhTruyen');
assert(numel(rods) == 6, 'can 6 thanh truyen, tim thay %d', numel(rods));
for k = 1:numel(rods)
    set_param(rods{k},'InertiaType','CalculateFromGeometry','BasedOnType','Density', ...
        'Density','VatLieu.rho_thanh','DensityUnits','kg/m^3');
end

%% 7. dong co, TCP, cam bien
mot = {'Revolute15','Revolute16','Revolute17'};
for i=1:3
    J = [m '/' mot{i}];
    set_param(J,'MotionActuationMode','InputMotion','TorqueActuationMode','ComputedTorque', ...
        'SensePosition','on','SenseTorqueForce','on');
    pos = get_param(J,'Position'); x = pos(1); y = pos(2);
    blk = @(lib,name,varargin) add_block(lib,sprintf('%s/%s_%d',m,name,i),'Position',varargin{:});
    hG  = blk('simulink/Math Operations/Gain','cmd_gain',[x-330 y-60 x-290 y-30], 'Gain',sprintf('G.s(%d)',i));
    hB  = blk('simulink/Math Operations/Bias','cmd_bias',[x-260 y-60 x-220 y-30], 'Bias',sprintf('G.q0(%d)-G.s(%d)*G.th0(%d)',i,i,i));
    hSP = blk('nesl_utility/Simulink-PS Converter','SPS',[x-180 y-60 x-150 y-30], 'Unit','deg', ...
        'FilteringAndDerivatives','filter','SimscapeFilterOrder','2','InputFilterTimeConstant','Motor.tau_filter');
    hPq = blk('nesl_utility/PS-Simulink Converter','PSq',[x+120 y-60 x+150 y-30],'Unit','deg');
    hPt = blk('nesl_utility/PS-Simulink Converter','PSt',[x+120 y+40 x+150 y+70],'Unit','N*m');
    hB2 = blk('simulink/Math Operations/Bias','meas_bias',[x+180 y-60 x+220 y-30],'Bias',sprintf('-G.q0(%d)',i));
    hG2 = blk('simulink/Math Operations/Gain','meas_gain',[x+250 y-60 x+290 y-30],'Gain',sprintf('G.s(%d)',i));
    hB3 = blk('simulink/Math Operations/Bias','meas_th',  [x+320 y-60 x+360 y-30],'Bias',sprintf('G.th0(%d)',i));
    jp = get_param(J,'PortHandles');
    L = @(a,b) add_line(m,a,b,'autorouting','on');
    L(ph1(hG,'Out'),ph1(hB,'In')); L(ph1(hB,'Out'),ph1(hSP,'In'));
    sp = get_param(hSP,'PortHandles'); L(sp.RConn(1), jp.LConn(2));
    pq = get_param(hPq,'PortHandles'); pt = get_param(hPt,'PortHandles');
    L(jp.RConn(2), pq.LConn(1)); L(jp.RConn(3), pt.LConn(1));
    L(pq.Outport(1),ph1(hB2,'In')); L(ph1(hB2,'Out'),ph1(hG2,'In')); L(ph1(hG2,'Out'),ph1(hB3,'In'));
    set_param(J,'Name',sprintf('Khop_DongCo_%d',i));
end

% khung TCP gan tren ban dong (huong trung World) + he toa do robot + cam bien TCP
pp = get_param([m '/Cau_Ban_1'],'PortHandles'); platPort = peerPort(pp.LConn(1));
hT = add_block('sm_lib/Frames and Transforms/Rigid Transform',[m '/TCP'],'Position',[100 900 140 940], ...
    'RotationMethod','RotationMatrix','RotationMatrix','G.tcpRot', ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','G.tcpLocal','TranslationCartesianOffsetUnits','m');
tp = get_param(hT,'PortHandles'); add_line(m, platPort, tp.LConn(1));
hO = add_block('sm_lib/Frames and Transforms/Rigid Transform',[m '/He_Toa_Do_Robot'],'Position',[100 1000 140 1040], ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','G.origin','TranslationCartesianOffsetUnits','m');
op = get_param(hO,'PortHandles');
hS = add_block('sm_lib/Frames and Transforms/Transform Sensor',[m '/Cam_Bien_TCP'],'Position',[200 900 260 980], ...
    'SenseX','on','SenseY','on','SenseZ','on');
sp = get_param(hS,'PortHandles');
wp = get_param([m '/World'],'PortHandles');
add_line(m, wp.RConn(1), op.LConn(1)); add_line(m, op.RConn(1), sp.LConn(1)); add_line(m, tp.RConn(1), sp.RConn(1));
xyzN = {'x','y','z'};
for k=1:3
    h = add_block('nesl_utility/PS-Simulink Converter',[m '/PS_tcp_' xyzN{k}],'Position',[300 880+40*k 330 900+40*k],'Unit','m');
    hp = get_param(h,'PortHandles'); add_line(m, sp.RConn(1+k), hp.LConn(1));
end

%% 8. gom subsystem
legs = { ...
 {'Hybrid_Motor_5','Part1_1','Mat_Bich_8mm_4','Mat_Bich_8mm_5','TrucRenM6_3','bi_mattrauM6_V2_11','bi_mattrauM6_V2_12', ...
  'Bimattrau_M6_V2_15','Bimattrau_M6_V2_16','ThanhTruyen_450mm_5','ThanhTruyen_450mm_6','Bimattrau_M6_V2_4','Bimattrau_M6_V2_9'}, ...
 {'Hybrid_Motor_3','Part1_2','Mat_Bich_8mm_6','Mat_Bich_8mm_8','TrucRenM6_2','bi_mattrauM6_V2_10','bi_mattrauM6_V2_6', ...
  'Bimattrau_M6_V2_14','Bimattrau_M6_V2_10','ThanhTruyen_450mm_4','ThanhTruyen_450mm_3','Bimattrau_M6_V2_5','Bimattrau_M6_V2_6'}, ...
 {'Hybrid_Motor_6','Part1_3','Mat_Bich_8mm_1','Mat_Bich_8mm_3','TrucRenM6_1','bi_mattrauM6_V2_2','bi_mattrauM6_V2_3','bi_mattrauM6_V2_4', ...
  'Bimattrau_M6_V2_1','Bimattrau_M6_V2_2','ThanhTruyen_450mm_2','ThanhTruyen_450mm_1','Bimattrau_M6_V2_8','Bimattrau_M6_V2_7'}};
legLocks = { {'Revolute6','Revolute22','Revolute23','Revolute2','Revolute3','Revolute9','Revolute14'}, ...
             {'Planar','Revolute20','Revolute21','Revolute1','Revolute','Revolute10','Revolute11'}, ...
             {'Cylindrical1','Revolute4','Revolute18','Revolute19','Cylindrical6','Revolute7','Revolute8','Revolute13','Revolute12'}};
for i=1:3
    names = [strcat(legs{i},'_RIGID'), strcat('Lock_',legLocks{i}), ...
             {sprintf('Cau_Tay_%d',2*i-1),sprintf('Cau_Tay_%d',2*i),sprintf('Cau_Ban_%d',2*i-1),sprintf('Cau_Ban_%d',2*i), ...
              sprintf('Khop_DongCo_%d',i)}, ...
             strcat({'cmd_gain_','cmd_bias_','SPS_','PSq_','PSt_','meas_bias_','meas_gain_','meas_th_'}, num2str(i))];
    group(m, names, sprintf('Chan_%d',i), [450 80+230*(i-1) 600 250+230*(i-1)]);
end
group(m, {'Ga_DinhMat_1_RIGID','Khung_1_RIGID','Ga_Hybrid_1_RIGID','Ga_Hybrid_2_RIGID','Ga_Hybrid2_1_RIGID'}, 'De', [250 250 350 450]);
group(m, {'Assem1_1_RIGID','TCP'}, 'Ban_Dong', [750 250 850 450]);
for s = {'Chan_1','Chan_2','Chan_3','De','Ban_Dong'}
    pruneUnconnected([m '/' s{1}]);
    bodies = find_system([m '/' s{1}],'SearchDepth',1,'RegExp','on','BlockType','SubSystem','Name','_RIGID$');
    for k = 1:numel(bodies), pruneBody(bodies{k}); end
end
renamePorts(m);

% dat ten cho dau vao/ra cua plant va gom thanh Robot_Delta
hDm = add_block('simulink/Signal Routing/Demux',[m '/Demux_cmd'],'Outputs','3','Position',[330 100 335 700]);
hIn = add_block('simulink/Sources/Constant',[m '/tmp_in'],'Value','[0;0;0]','Position',[250 385 280 415]);
add_line(m, ph1(hIn,'Out'), ph1(hDm,'In'));
muxes = {'Mux_theta','Mux_torque','Mux_tcp'}; tmps = {};
for k=1:3
    h = add_block('simulink/Signal Routing/Mux',[m '/' muxes{k}],'Inputs','3','Position',[1000 100+250*(k-1) 1005 300+250*(k-1)]);
    t = add_block('simulink/Sinks/Terminator',[m '/tmp_out' num2str(k)],'Position',[1060 190+250*(k-1) 1080 210+250*(k-1)]);
    add_line(m, ph1(h,'Out'), ph1(t,'In')); tmps{end+1} = t; %#ok<AGROW>
end
for i=1:3
    cp = get_param([m '/Chan_' num2str(i)],'PortHandles');
    add_line(m, dm(hDm,i), portByName(m,['Chan_' num2str(i)],'Inport','theta_dat'),'autorouting','on');
    add_line(m, portByName(m,['Chan_' num2str(i)],'Outport','theta_do'), mx(m,'Mux_theta',i),'autorouting','on');
    add_line(m, portByName(m,['Chan_' num2str(i)],'Outport','momen'),   mx(m,'Mux_torque',i),'autorouting','on');
    add_line(m, ph1([m '/PS_tcp_' xyzN{i}],'Out'), mx(m,'Mux_tcp',i),'autorouting','on');
end
set_param([m '/Solver' newline 'Configuration'],'Name','Solver_Configuration');
all = find_system(m,'SearchDepth',1,'Type','block');
all = setdiff(all, {[m '/tmp_in'],[m '/tmp_out1'],[m '/tmp_out2'],[m '/tmp_out3']});
group(m, strrep(all, [m '/'], ''), 'Robot_Delta', [520 120 720 360]);
P = [m '/Robot_Delta'];
renameIO(P, 'Inport',  {'Demux_cmd'}, {'theta_dat'});
renameIO(P, 'Outport', muxes, {'theta_do','momen','tcp'});
for b = {'tmp_in','tmp_out1','tmp_out2','tmp_out3'}
    lh = get_param([m '/' b{1}],'LineHandles'); lh = [lh.Inport lh.Outport]; delete_line(lh(lh>0)); delete_block([m '/' b{1}]);
end
pruneUnconnected(P); tryArrange(P);

%% 9. tang tren: Quy dao -> IK -> Robot -> Scope
hC  = add_block('simulink/Sources/Clock',[m '/Clock'],'Position',[40 225 60 245]);
hTr = addMatlabFcn([m '/Quy_Dao'], [120 200 260 270], trajCode(), 'Traj');
hIK = addMatlabFcn([m '/IK'],      [320 200 440 270], ...
    sprintf('function theta_dat = IK(xyz, G)\n%% Dong hoc nguoc: TCP [m] -> goc canh tay [deg] (xem lib/RobotDelta_IK.m)\ntheta_dat = RobotDelta_IK(xyz, G);\n'), 'G');
set_param(P,'Position',[520 180 700 290]);
add_line(m, ph1(hC,'Out'), ph1(hTr,'In'));
add_line(m, ph1(hTr,'Out'), ph1(hIK,'In'));
add_line(m, ph1(hIK,'Out'), ph1(P,'In'));
Pp = get_param(P,'PortHandles');
sc = @(name,n,pos) add_block('simulink/Sinks/Scope',[m '/' name],'NumInputPorts',num2str(n),'Position',pos);
hS1 = sc('Scope_Goc',   2,[900 60 940 120]);
hS2 = sc('Scope_MoMen', 1,[900 180 940 220]);
hS3 = sc('Scope_TCP',   2,[900 280 940 340]);
s1 = get_param(hS1,'PortHandles'); s3 = get_param(hS3,'PortHandles');
add_line(m, ph1(hIK,'Out'), s1.Inport(1),'autorouting','on');
add_line(m, Pp.Outport(1), s1.Inport(2),'autorouting','on');
add_line(m, Pp.Outport(2), ph1(hS2,'In'),'autorouting','on');
add_line(m, ph1(hTr,'Out'), s3.Inport(1),'autorouting','on');
add_line(m, Pp.Outport(3), s3.Inport(2),'autorouting','on');
names = {'theta_dat','theta_do','momen','tcp_dat','tcp'};
srcs  = {ph1(hIK,'Out'), Pp.Outport(1), Pp.Outport(2), ph1(hTr,'Out'), Pp.Outport(3)};
for k=1:5
    h = add_block('simulink/Sinks/To Workspace',[m '/log_' names{k}],'VariableName',names{k}, ...
        'SaveFormat','Timeseries','Position',[1000 40+70*k 1080 60+70*k]);
    add_line(m, srcs{k}, ph1(h,'In'),'autorouting','on');
    set_param(get_param(srcs{k},'Line'),'Name',names{k});
end
tryArrange(m);

preload = ['if isempty(which(''RobotDelta_Params'')), ' ...
           'run(fullfile(fileparts(get_param(bdroot,''FileName'')),''..'',''tools'',''rd_addpath.m'')); end; ' ...
           'RobotDelta_Params;'];
set_param(m,'PreLoadFcn',preload,'InitFcn','RobotDelta_Params;', ...
    'StopTime','10','Solver','daessc','MaxStep','1e-2','SimscapeLogType','none');
save_system(m);
fprintf('\n==> Da tao %s\n', fullfile(mdlDir,[m '.slx']));
fprintf('\n--- Kiem tra file CAD them tay (cad/solidworks) ---\n');
rd_checkgeom();

%% 10. kiem tra
validate(m, G);
close_system(m,1);
end

%% ======================= do hinh hoc =======================
function G = measureGeometry(mdlDir, m, qMot)
tmpFile = fullfile(tempdir,'RD_measure.slx');
close_system(m,1); copyfile(fullfile(mdlDir,[m '.slx']), tmpFile,'f');
t = 'RD_measure'; load_system(tmpFile);
pp = get_param([t '/Cau_Ban_1'],'PortHandles'); platPort = peerPort(pp.LConn(1));
for k=1:3
    o = zeros(1,3); o(k) = 0.1;
    b = add_block('sm_lib/Frames and Transforms/Rigid Transform',sprintf('%s/Tmp%d',t,k), ...
        'TranslationMethod','Cartesian','TranslationCartesianOffset',mat2str(o),'TranslationCartesianOffsetUnits','m');
    p = get_param(b,'PortHandles'); add_line(t,platPort,p.LConn(1));
end
ks = simscape.multibody.KinematicsSolver(t);
W = [t '/World/W'];
addFrameVariables(ks,'O','Translation',W,[t '/Cau_Ban_1/B']);
for k=1:3, addFrameVariables(ks,sprintf('T%d',k),'Translation',W,sprintf('%s/Tmp%d/F',t,k)); end
for i=1:6
    addFrameVariables(ks,sprintf('A%d',i),'Translation',W,sprintf('%s/Cau_Tay_%d/B',t,i));
    addFrameVariables(ks,sprintf('P%d',i),'Translation',W,sprintf('%s/Cau_Ban_%d/B',t,i));
end
jv = jointPositionVariables(ks);
mot = {'Revolute15','Revolute16','Revolute17'};
ids = arrayfun(@(i) jv.ID(strcmp(jv.BlockPath,[t '/' mot{i}])), 1:3);
addTargetVariables(ks, ids); fv = frameVariables(ks); addOutputVariables(ks, fv.ID);
S0 = pts(ks, qMot); Sp = pts(ks, qMot+5); Sn = pts(ks, qMot-5);
close_system(t,0); delete(tmpFile); load_system(fullfile(mdlDir,[m '.slx']));

G.q0 = qMot;
tcp0 = mean(S0.P,1);
R = [(S0.T1-S0.O)' (S0.T2-S0.O)' (S0.T3-S0.O)']/0.1;       % huong ban dong trong World
G.tcpLocal = (R'*(tcp0-S0.O)')'; G.tcpRot = R';
for i=1:3
    pr = [2*i-1 2*i];
    A0 = mean(S0.A(pr,:),1); Ap = mean(Sp.A(pr,:),1); An = mean(Sn.A(pr,:),1);
    a = Ap-A0; b = An-A0; n = cross(a,b);
    F = A0 + cross(dot(a,a)*b - dot(b,b)*a, n)/(2*dot(n,n));  % tam quay = diem tren truc dong co
    d = n/norm(n);
    e1 = cross([0 0 1], d); e1 = e1/norm(e1); if dot(e1, A0-F) < 0, e1 = -e1; end
    ez = cross(d, e1);      if ez(3) < 0, ez = -ez; end
    th = @(A) atan2d(-dot(A-F,ez), dot(A-F,e1));
    G.F(i,:) = F; G.e1(i,:) = e1; G.ez(i,:) = ez;
    G.La(i) = norm(A0-F);
    G.th0(i) = th(A0);
    G.s(i) = round((th(Ap)-th(A0))/5);
    Pm = mean(S0.P(pr,:),1);
    G.Lr(i) = norm(A0-Pm);
    G.b(i,:) = Pm - tcp0;
    G.wArm(i)  = norm(diff(S0.A(pr,:)));     % khoang cach 2 bi tren canh tay
    G.wPlat(i) = norm(diff(S0.P(pr,:)));     % khoang cach 2 bi tren ban dong
    assert(abs(abs(th(Ap)-th(A0))-5) < 1e-6, 'truc dong co %d khong vuong goc canh tay', i);
end
G.branch = [1 1 1]; t1 = RobotDelta_IK(tcp0, G)';
G.branch(abs(t1 - G.th0) > 1e-3) = -1;
t2 = RobotDelta_IK(tcp0, G)';
assert(max(abs(t2-G.th0)) < 1e-3, 'IK khong khop tu the CAD');
% he toa do robot: goc = diem gan nhat toi 3 mat phang canh tay (XY), o cao do truc dong co
M = zeros(2); v = zeros(2,1);
for i=1:3
    u = G.e1(i,1:2)'/norm(G.e1(i,1:2)); Pn = eye(2) - u*u';
    M = M + Pn; v = v + Pn*G.F(i,1:2)';
end
G.origin  = [(M\v)' mean(G.F(:,3))];
G.F       = G.F - G.origin;
G.tcp_cad = tcp0 - G.origin;
G.home    = [0 0 G.tcp_cad(3)];
end

function adjustArmSpacing(m, qMot, target)
% Moi bi tren canh tay gan vao truc ren qua 1 khop da khoa (Lock_Revolute..), truc Z cua khop
% = truc ren. Dich tinh tien theo Z de 2 bi cach nhau dung 'target'. Chieu dich do bang
% KinematicsSolver (dich thu 1 mm tung bi), roi kiem tra lai.
locks = {'Lock_Revolute22','Lock_Revolute23'; 'Lock_Revolute20','Lock_Revolute21'; 'Lock_Revolute18','Lock_Revolute19'};
off = @(b) str2num(get_param([m '/' b],'TranslationCartesianOffset')); %#ok<ST2NM>
setz = @(b, dz) set_param([m '/' b],'TranslationCartesianOffset',mat2str(off(b) + [0 0 dz],17));
w0 = armSpacing(m, qMot);
k = zeros(3,2);
for c = 1:2
    for i = 1:3, setz(locks{i,c}, 1e-3); end
    k(:,c) = (armSpacing(m, qMot) - w0)/1e-3;             % mm spacing / mm dich
    for i = 1:3, setz(locks{i,c}, -1e-3); end
end
assert(all(abs(abs(k(:)) - 1) < 1e-3), 'truc khop khoa khong trung truc ren: %s', mat2str(k,4));
for i = 1:3
    d = (target - w0(i))/2;
    setz(locks{i,1}, d/k(i,1)); setz(locks{i,2}, d/k(i,2));
end
w = armSpacing(m, qMot);
fprintf('Khoang cach 2 bi tren canh tay: %s mm -> %s mm\n', mat2str(round(w0*1e3,3)), mat2str(round(w*1e3,3)));
assert(all(abs(w - target) < 1e-6), 'khong dat khoang cach bi mong muon');
end

function w = armSpacing(m, qMot)
ks = simscape.multibody.KinematicsSolver(m);
for i = 1:6, addFrameVariables(ks,sprintf('A%d',i),'Translation',[m '/World/W'],sprintf('%s/Cau_Tay_%d/B',m,i)); end
jv = jointPositionVariables(ks);
ids = arrayfun(@(i) jv.ID(strcmp(jv.BlockPath,[m '/Revolute1' num2str(4+i)])), 1:3);
addTargetVariables(ks, ids); fv = frameVariables(ks); addOutputVariables(ks, fv.ID);
[o, st] = solve(ks, qMot(:)); assert(st==1, 'KinematicsSolver that bai');
ov = outputVariables(ks); get = @(n) arrayfun(@(c) o(strcmp(ov.ID, [n '.Translation.' c])), 'xyz');
w = zeros(3,1);
for i = 1:3, w(i) = norm(get(sprintf('A%d',2*i)) - get(sprintf('A%d',2*i-1))); end
end

function S = pts(ks, q)
[o, st] = solve(ks, q(:)); assert(st==1, 'KinematicsSolver that bai');
ov = outputVariables(ks); get = @(n) arrayfun(@(c) o(strcmp(ov.ID, [n '.Translation.' c])), 'xyz');
S.O = get('O'); for k=1:3, S.(sprintf('T%d',k)) = get(sprintf('T%d',k)); end
for i=1:6, S.A(i,:) = get(sprintf('A%d',i)); S.P(i,:) = get(sprintf('P%d',i)); end
end

function writeGeometryFile(fn, G)
f = fopen(fn,'w'); c = onCleanup(@() fclose(f));
fprintf(f,'%%%% RobotDelta_Geometry.m -- TU SINH boi build_RobotDelta_Sim.m (%s). KHONG sua tay.\n', datestr(now,'yyyy-mm-dd HH:MM'));
fprintf(f,'%% Hinh hoc dong hoc do tu CAD, don vi m / deg. Moi hang = 1 chan.\n');
fprintf(f,'%% He toa do robot: goc = tam de, o cao do truc dong co, z huong len (truc song song World).\n');
w = @(name, val, cmt) fprintf(f,'%-12s = %s;  %% %s\n', name, mat2str(val,10), cmt);
w('G.origin', G.origin, 'goc he toa do robot, tinh trong he World cua CAD');
w('G.La',     G.La,     'chieu dai canh tay tren (truc dong co -> tam 2 bi)');
w('G.Lr',     G.Lr,     'chieu dai thanh truyen (tam bi -> tam bi)');
w('G.wArm',   G.wArm,   'khoang cach 2 bi tren canh tay');
w('G.wPlat',  G.wPlat,  'khoang cach 2 bi tren ban dong (= wArm neu la hinh binh hanh)');
w('G.F',      G.F,      'diem tren truc dong co (mat phang canh tay)');
w('G.e1',     G.e1,     'phuong ngang huong ra ngoai cua canh tay');
w('G.ez',     G.ez,     'phuong "len" trong mat phang canh tay');
w('G.b',      G.b,      'vi tri tam cap bi tren ban dong so voi TCP');
w('G.q0',     G.q0,     'goc khop dong co tai tu the CAD [deg]');
w('G.th0',    G.th0,    'goc canh tay duoi phuong ngang tai tu the CAD [deg]');
w('G.s',      G.s,      'chieu: theta = th0 + s*(q - q0)');
w('G.branch', G.branch, 'nhanh nghiem IK');
w('G.tcpLocal',G.tcpLocal,'TCP trong khung ban dong');
w('G.tcpRot', G.tcpRot, 'xoay TCP cho trung huong World');
w('G.tcp_cad',G.tcp_cad,'TCP tai tu the CAD');
w('G.home',   G.home,   'vi tri nghi (tam vung lam viec)');
end

%% ======================= kiem tra =======================
function validate(m, G)
fprintf('\n--- Kiem tra IK voi KinematicsSolver ---\n');
ks = simscape.multibody.KinematicsSolver(m);
P = [m '/Robot_Delta'];
addFrameVariables(ks,'TCP','Translation',[P '/He_Toa_Do_Robot/F'],[P '/Ban_Dong/TCP/F']);
jv = jointPositionVariables(ks);
ids = arrayfun(@(i) jv.ID(contains(jv.BlockPath, sprintf('Khop_DongCo_%d',i))), 1:3);
addTargetVariables(ks, ids); addOutputVariables(ks, frameVariables(ks).ID);
tests = G.home + [0 0 0; 0.06 0 0; -0.06 0 0; 0 0.06 0; 0 -0.06 0; 0 0 0.04; 0 0 -0.04; 0.05 0.05 0.03];
for k=1:size(tests,1)
    th = RobotDelta_IK(tests(k,:), G)';
    q = G.q0 + G.s.*(th - G.th0);
    [o, st] = solve(ks, q(:));
    fprintf('  dat %s mm -> theta %s deg -> thuc %s mm | sai %.3f mm (st=%d)\n', mat2str(round(tests(k,:)*1000,1)), ...
        mat2str(round(th,2)), mat2str(round(o'*1000,1)), norm(o'-tests(k,:))*1000, st);
end
fprintf('\n--- Mo phong 10 s (duong tron) ---\n');
try, set_param(m,'SimMechanicsOpenEditorOnUpdate','off'); catch, end
out = sim(m);
try, set_param(m,'SimMechanicsOpenEditorOnUpdate','on'); catch, end
r = out.tcp_dat; y = out.tcp; tq = out.momen;
yr = interp1(r.Time, nx3(r.Data), y.Time); e = vecnorm(nx3(y.Data) - yr, 2, 2);
fprintf('  sai so TCP: max %.2f mm (sau 1.5 s: %.2f mm)\n', max(e)*1000, max(e(y.Time>1.5))*1000);
fprintf('  mo-men dong co max: %s N*m\n', mat2str(round(max(abs(nx3(tq.Data)),[],1),3)));
end

%% ======================= tien ich =======================
function c = trajCode()
c = strjoin({ ...
'function xyz = Quy_Dao(t, Traj)'
'% Quy dao TCP [m], he toa do robot. Ham dung chung voi robot that: lib/RobotDelta_Traj.m'
'% Thong so: Traj trong params/RobotDelta_Params.m (mode 0 dung yen | 1 tron | 2 gap-dat | 3 danh sach diem)'
'xyz = RobotDelta_Traj(t, Traj);'
'end'}, newline);
end

function h = addMatlabFcn(path, pos, code, param)
h = add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart','Path',path);
ch.Script = code;
d = ch.find('-isa','Stateflow.Data','Name',param); d.Scope = 'Parameter';
end

function group(m, names, newName, pos)
h = cellfun(@(n) get_param([m '/' n],'Handle'), names);
Simulink.BlockDiagram.createSubsystem(h);
sub = get_param(h(1),'Parent');
set_param(sub,'Name',newName,'Position',pos);
tryArrange([m '/' newName]);
end

function pruneUnconnected(S)
% xoa cong vat ly cua subsystem ma ben ngoai khong noi gi (frame thua cua CAD)
ph = get_param(S,'PortHandles');
pm = find_system(S,'SearchDepth',1,'BlockType','PMIOPort');
[~,ord] = sort(cellfun(@(b) str2double(get_param(b,'Port')), pm)); pm = pm(ord);
isL = strcmp(cellfun(@(b) get_param(b,'Side'), pm, 'uni', 0), 'Left');
assert(nnz(isL)==numel(ph.LConn) && nnz(~isL)==numel(ph.RConn), 'khong khop cong %s', S);
outer = zeros(size(pm)); outer(isL) = ph.LConn; outer(~isL) = ph.RConn;
dead = pm(arrayfun(@(p) get_param(p,'Line') == -1, outer));
for k=1:numel(dead)
    lh = get_param(dead{k},'LineHandles'); lh = [lh.LConn lh.RConn]; delete_line(lh(lh>0)); delete_block(dead{k});
end
tryArrange(S);
end

function pruneBody(S)
% chi tiet CAD: xoa cong + Rigid Transform cua cac frame khong con dung (moi han / khop da go)
ph = get_param(S,'PortHandles');
if all(arrayfun(@(p) get_param(p,'Line') ~= -1, [ph.LConn ph.RConn])), return; end
pruneUnconnected(S);
rt = find_system(S,'SearchDepth',1,'RegExp','on','ReferenceBlock','Rigid.Transform$');
for k = 1:numel(rt)
    p = get_param(rt{k},'PortHandles');
    if get_param(p.RConn(1),'Line') == -1
        lh = get_param(rt{k},'LineHandles'); lh = [lh.LConn lh.RConn]; delete_line(lh(lh>0)); delete_block(rt{k});
    end
end
end

function renamePorts(m)
% dat ten cong ket noi vat ly theo khoi ben trong ma no noi toi
for s = {'Chan_1','Chan_2','Chan_3','De','Ban_Dong'}
    S = [m '/' s{1}];
    pm = find_system(S,'SearchDepth',1,'BlockType','PMIOPort');
    for k=1:numel(pm)
        n = peerNames1(pm{k});
        if strcmp(s{1},'Ban_Dong'), if contains(n,'TCP'), nm = 'TCP'; else, nm = 'Noi_Chan'; end
        elseif contains(n,'Hybrid_Motor') || contains(n,'Ga_Hybrid'), nm = 'Noi_De';
        elseif contains(n,'Ga_DinhMat'), nm = 'Noi_World';
        elseif contains(n,'Cau_Ban') || contains(n,'Assem1'), nm = 'Noi_BanDong';
        else, nm = 'Conn'; end
        set_param(pm{k},'Name',[nm '_' num2str(k)]);
    end
    ip = find_system(S,'SearchDepth',1,'BlockType','Inport');
    for k=1:numel(ip), set_param(ip{k},'Name','theta_dat'); end
    op = find_system(S,'SearchDepth',1,'BlockType','Outport');
    for k=1:numel(op)
        n = peerNames1(op{k});
        if contains(n,'meas_th'), set_param(op{k},'Name','theta_do'); else, set_param(op{k},'Name','momen'); end
    end
end
end

function renameIO(P, type, peers, newNames)
b = find_system(P,'SearchDepth',1,'BlockType',type);
for k=1:numel(b)
    n = peerNames1(b{k});
    idx = find(cellfun(@(x) contains(n,x), peers), 1);
    if ~isempty(idx), set_param(b{k},'Name',newNames{idx}); end
end
end

function n = peerNames1(blk)
ph = get_param(blk,'PortHandles'); all = [ph.Inport ph.Outport ph.LConn ph.RConn];
n = '';
for p = all
    l = get_param(p,'Line'); if l<0, continue; end
    e = [get_param(l,'SrcPortHandle') get_param(l,'DstPortHandle')]; e = e(e>0 & e~=p);
    for q = e, n = [n ' ' get_param(get_param(q,'Parent'),'Name')]; end %#ok<AGROW>
end
end

function p = portByName(m, sub, type, name)
b = find_system([m '/' sub],'SearchDepth',1,'BlockType',type,'Name',name);
num = str2double(get_param(b{1},'Port'));
ph = get_param([m '/' sub],'PortHandles');
if strcmp(type,'Inport'), p = ph.Inport(num); else, p = ph.Outport(num); end
end

function p = dm(h,i), ph = get_param(h,'PortHandles'); p = ph.Outport(i); end
function p = mx(m,name,i), ph = get_param([m '/' name],'PortHandles'); p = ph.Inport(i); end
function p = ph1(h, kind)
ph = get_param(h,'PortHandles');
if strcmp(kind,'In'), p = ph.Inport(1); else, p = ph.Outport(1); end
end
function tryArrange(s)
try, Simulink.BlockDiagram.arrangeSystem(s); catch, end
end
function n = peerNames(l,p)
e = [get_param(l,'SrcPortHandle') get_param(l,'DstPortHandle')]; e = e(e>0 & e~=p);
n = arrayfun(@(h) get_param(get_param(h,'Parent'),'Name'), e, 'uni', 0);
end
function o = peerPort(p)
l = get_param(p,'Line'); e = [get_param(l,'SrcPortHandle') get_param(l,'DstPortHandle')];
o = e(e>0 & e~=p); o = o(1);
end
function [pb,pf,pos] = joinPeers(m,j)
b = [m '/' j]; ph = get_param(b,'PortHandles'); pos = get_param(b,'Position');
pb = cutPeer(ph.LConn(1)); pf = cutPeer(ph.RConn(1));
end
function o = cutPeer(p)
l = get_param(p,'Line'); e = [get_param(l,'SrcPortHandle') get_param(l,'DstPortHandle')];
o = e(e>0 & e~=p); assert(numel(o)==1,'duong noi re nhanh'); delete_line(l);
end
function lockJoint(m,j,angDeg,tr)
[pb,pf,pos] = joinPeers(m,j); delete_block([m '/' j]);
rt = add_block('sm_lib/Frames and Transforms/Rigid Transform',[m '/Lock_' j],'Position',pos, ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+Z','RotationAngle',num2str(angDeg,17), ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset',mat2str(tr,17),'TranslationCartesianOffsetUnits','m');
ph = get_param(rt,'PortHandles'); add_line(m,pb,ph.LConn(1)); add_line(m,ph.RConn(1),pf);
end
function weld2sph(m,bBlk,fBlk,name)
ph = get_param([m '/' bBlk],'PortHandles');
for p = [ph.LConn ph.RConn]
    l = get_param(p,'Line'); if l<0, continue; end
    e = [get_param(l,'SrcPortHandle') get_param(l,'DstPortHandle')]; o = e(e>0 & e~=p);
    if isscalar(o) && strcmp(get_param(get_param(o,'Parent'),'Name'),fBlk)
        delete_line(l); pos = get_param([m '/' fBlk],'Position');
        s = add_block('sm_lib/Joints/Spherical Joint',[m '/' name],'Position',pos+[-80 0 -80 0]);
        sp = get_param(s,'PortHandles'); add_line(m,p,sp.LConn(1)); add_line(m,sp.RConn(1),o);
        return
    end
end
error('Khong tim thay moi han %s - %s', bBlk, fBlk);
end
function v = first(x), v = x(1); end
function d = nx3(d), if ndims(d)==3, d = squeeze(d)'; end, end
function u = unitOf(f), if f=='q', u = 'deg'; else, u = 'm'; end, end
