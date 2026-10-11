%% RobotDelta_Params.m  --  THONG SO chung cho mo phong va thuc nghiem
% Tu chay khi mo model (PreLoadFcn) va moi lan bam Run (InitFcn).
% Chi can sua cac gia tri trong phan "NGUOI DUNG SUA" ben duoi.
%
% He toa do robot: goc O = tam de, o cao do truc dong co; z huong LEN (ban dong nam o z < 0).

RobotDelta_DataFile;   % smiData : khoi luong, quan tinh cac chi tiet tu SolidWorks (khong sua)
RobotDelta_Geometry;   % G       : hinh hoc dong hoc do tu CAD (tu sinh boi tools/build_RobotDelta_Sim.m)

%% ====================== NGUOI DUNG SUA ======================

% --- Quy dao TCP (xem lib/RobotDelta_Traj.m) ---
%   0 dung yen | 1 duong tron | 2 gap-dat A <-> B | 3 di qua danh sach diem
Traj.mode     = 1;
Traj.home     = G.home;                  % [m] vi tri nghi (giua vung lam viec)
Traj.t_ramp   = 1.0;                     % [s] mode 1, 2: thoi gian di tu tu tu home vao quy dao
Traj.radius   = 0.06;                    % [m] mode 1
Traj.period   = 2.0;                     % [s] mode 1: 1 vong | mode 2: 1 chu ky A->B->A
Traj.pA       = G.home + [-0.06 0 0.02]; % [m] mode 2
Traj.pB       = G.home + [ 0.06 0 0.02]; % [m] mode 2
Traj.lift     = 0.04;                    % [m] mode 2: do cao nhac
Traj.wp       = G.home + [0.05 0 0; -0.05 0 0];   % [m] mode 3: moi hang 1 diem
Traj.wp_T     = 1.0;                     % [s] mode 3: thoi gian di moi doan
Traj.wp_dwell = 0.5;                     % [s] mode 3: thoi gian dung tai moi diem

% --- Dong co & driver ---
% Dong co buoc lai co encoder + driver HDB CZ880 + hop so 1:10 (do ngay 30/09/2026, solieu_encoder/)
Motor.steps_rev   = 200;    % buoc / vong cua dong co (1.8 deg/buoc)
Motor.microstep   = 4;      % driver dat 800 xung / vong truc dong co = 200 x 4 (DIP SW1-SW4)
Motor.gear        = 10;     % hop so 1:10 -> 1 vong canh tay = 10 vong dong co = 8000 xung
Motor.encoder_ppr = 1000;   % encoder 1000 vach -> 4000 dem/vong dong co -> 40000 dem/vong canh tay
% Dong co HS57-64, than dai 76 mm (do tren CAD 11/10/2026) -> hang 57HS76. CHUA CO DATASHEET dung ma:
% so lieu duoi la gia tri dien hinh cua dong 57 mm dai 76 mm -> thay khi co datasheet / do thuc nghiem
Motor.T_hold      = 2.0;    % [N*m] mo-men giu (dien hinh 2.0-2.2, lay can duoi)
Motor.J_rotor     = 4.8e-5; % [kg*m^2] quan tinh roto (dien hinh 480 g*cm^2). Quy ve canh tay x gear^2 = 4.8e-3
Motor.eta_gear    = 0.9;    % hieu suat hop so hanh tinh 57PX10 (1 cap: 0.90-0.92)
Motor.T_gear_max  = 12;     % [N*m] mo-men ra cho phep cua hop so (dien hinh dong 57 mm 10:1)

% --- Vat lieu ---
VatLieu.rho_thanh = 2700;   % [kg/m^3] 6 thanh truyen: nhom (thep ~7850). Khoi luong tinh tu hinh hoc STEP

% --- Gioi han an toan (KIEM TRA / DO LAI tren robot that) ---
Lim.theta       = [-20 80];   % [deg] goc tay cho phep (tranh va cham de / thanh truyen)
Lim.tilt_rodend = 13;         % [deg] goc nghieng toi da cua bi mat trau M6 (tra datasheet)
Lim.gamma       = [20 160];   % [deg] goc giua canh tay va thanh truyen (xa vung ky di)
Lim.omega       = NaN;        % [deg/s] toc do toi da cua dong co   (NaN = chua khai bao)
Lim.torque      = 8.2;        % [N*m]   mo-men cho phep tai truc canh tay (= Nmpc.tau_max, xem duoi)

% --- Bo dieu khien du bao NMPC (nmpc/, phuong an A: NMPC xuat goc/van toc dat cho driver) ---
Nmpc.Ts        = 0.01;       % [s] chu ky dieu khien (100 Hz)
Nmpc.N         = 20;         % so buoc du bao -> tam nhin N*Ts = 0.2 s
Nmpc.Wq        = 1e5;        % trong so sai so goc
Nmpc.Wdq       = 10;         % trong so sai so van toc goc
Nmpc.Wu        = 1e-3;       % trong so gia toc goc (dau vao) -> muot
% mo-men cho phep tai truc canh tay = min(T_hold * 0.5 * gear * eta, T_gear_max) / 1.1
%   0.5: mo-men con lai o ~1000 v/ph dong co (toc do lon nhat) | 1.1: du 10 % cho sai so mo hinh + RTI
Nmpc.tau_max   = min(Motor.T_hold*0.5*Motor.gear*Motor.eta_gear, Motor.T_gear_max)/1.1;   % = 8.2 N*m
Nmpc.omega_max = 600;        % [deg/s] toc do canh tay (600 deg/s = 1000 v/ph dong co = 13.3 kHz xung)
Nmpc.acc_max   = 2e4;        % [deg/s^2] gia toc canh tay

%% ============================================================
RobotDelta_DynParams;  % Dyn.phi : tham so dong luc hoc rut gon (tu sinh boi tools/rd_identdyn.m)
