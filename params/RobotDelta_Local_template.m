%% RobotDelta_Local.m  --  CAI DAT RIENG CUA MAY NAY (khong chia se, khong dua len Git)
% Lan dau mo project tren 1 may, file nay duoc copy tu params/RobotDelta_Local_template.m
% sang local/RobotDelta_Local.m. Moi may giu ban rieng cua minh -> sua thoai mai.

% Cong Serial noi vi dieu khien robot. Xem cac cong dang co: serialportlist("available")
%   Windows "COM3" | Linux "/dev/ttyUSB0" hoac "/dev/ttyACM0" | macOS "/dev/tty.usbserial-XXXX"
Local.port = "COM3";

% Ten nguoi chay thi nghiem (gan vao ten thu muc ket qua). "" = ten dang nhap cua may.
Local.user = "";

% Thu muc luu ket qua thi nghiem. "" = <du an>/data (chi o may nay).
% Ca nhom dung chung ket qua: tro toi 1 thu muc dong bo, vd:
%   "G:\My Drive\RobotDelta_data"   (Google Drive)   |   "D:\OneDrive\RobotDelta_data"
Local.dataDir = "";

% NMPC: thu muc acados va CasADi (tools/rd_acados.m). "" = tu tim o ..\tools_ext\ canh du an.
%   Windows can them CMake: de "" de tu tim CMake di kem Visual Studio 2022.
Local.acadosDir = "";
Local.casadiDir = "";
Local.cmakeDir  = "";
