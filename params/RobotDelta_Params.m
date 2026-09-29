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
Motor.steps_rev  = 200;     % buoc / vong cua dong co (1.8 deg/buoc)
Motor.microstep  = 16;      % vi buoc cai tren driver
Motor.gear       = 1;       % ti so truyen dong co -> canh tay (1 = noi truc tiep)

% --- Vat lieu ---
VatLieu.rho_thanh = 2700;   % [kg/m^3] 6 thanh truyen: nhom (thep ~7850). Khoi luong tinh tu hinh hoc STEP

% --- Gioi han an toan (KIEM TRA / DO LAI tren robot that) ---
Lim.theta       = [-20 80];   % [deg] goc tay cho phep (tranh va cham de / thanh truyen)
Lim.tilt_rodend = 13;         % [deg] goc nghieng toi da cua bi mat trau M6 (tra datasheet)
Lim.gamma       = [20 160];   % [deg] goc giua canh tay va thanh truyen (xa vung ky di)
Lim.omega       = NaN;        % [deg/s] toc do toi da cua dong co   (NaN = chua khai bao)
Lim.torque      = NaN;        % [N*m]   mo-men cho phep cua dong co (NaN = chua khai bao)

%% ============================================================
