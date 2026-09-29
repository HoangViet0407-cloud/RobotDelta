%% RobotDelta_Real.m  --  HINH HOC & KET NOI ROBOT THAT (dung cho exp_run(..., 'real'))
% Da do tren robot (2026-09-28): kich thuoc thuc te GIONG CAD, ke ca 3 truc dong co cach tam
% 32.3 / 48.8 / 81.4 mm (khong doi xung) -> robot that dung chinh hinh hoc CAD: Greal = G.
% He toa do robot: goc = tam de, o cao do truc dong co, z huong len.

if ~exist('G','var'), RobotDelta_Params; end

% --- Hinh hoc ---
Real.dung_CAD = true;          % true: Greal = G (CAD) | false: robot doi xung dung cac kich thuoc ben duoi
Real.R      = 0.0541;          % [m] (chi dung khi dung_CAD = false) tam de -> truc dong co
Real.r      = 0.0300;          % [m] tam ban dong -> diem giua cap bi
Real.La     = 0.1942;          % [m] truc dong co -> diem giua cap bi tren canh tay
Real.Lr     = 0.3742;          % [m] tam bi -> tam bi cua thanh truyen
Real.phi    = [150 30 270];    % [deg] huong ra ngoai cua canh tay 1, 2, 3
Real.h_tool = 0;               % [m] tu mat phang tam bi xuong dau cong cu (but / giac hut)
Real.z_home = G.home(3);       % [m] cao do vi tri nghi

% --- Hieu chuan dong co (firmware dung de doi goc -> buoc) ---
Real.theta_home = [NaN NaN NaN];   % [deg] goc tay khi cham cong tac home (do bang thuoc do goc)

% --- Ket noi vi dieu khien ---
Lc = rd_local();
Real.port = Lc.port;           % cong Serial: RIENG tung may -> sua trong local/RobotDelta_Local.m
Real.baud = 115200;

if Real.dung_CAD
    Greal = G;
else
    Greal = RobotDelta_GeomIdeal(Real, G);
end
