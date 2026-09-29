function L = rd_local()
%RD_LOCAL  Cai dat RIENG cua may dang chay: cong Serial, ten nguoi dung, thu muc du lieu.
%   L = rd_local()   ->  L.port, L.user, L.dataDir, L.may
%   Doc local/RobotDelta_Local.m (moi may 1 ban, khong chia se). Neu chua co, tu tao tu
%   params/RobotDelta_Local_template.m.

root = rd_root();
f = fullfile(root, 'local', 'RobotDelta_Local.m');
if ~isfile(f)
    if ~isfolder(fileparts(f)), mkdir(fileparts(f)); end
    copyfile(fullfile(root, 'params', 'RobotDelta_Local_template.m'), f);
    fprintf('Da tao cai dat rieng cho may nay: %s\n  -> sua cong Serial / thu muc du lieu neu can.\n', f);
end
Local = struct();
run(f);
L = Local;
if ~isfield(L,'port'), L.port = ""; end
if ~isfield(L,'user') || strlength(L.user) == 0
    L.user = string(getenv('USERNAME'));
    if strlength(L.user) == 0, L.user = string(getenv('USER')); end
end
if ~isfield(L,'dataDir') || strlength(L.dataDir) == 0
    L.dataDir = string(fullfile(root, 'data'));
end
L.may = string(getenv('COMPUTERNAME'));
if strlength(L.may) == 0, L.may = string(getenv('HOSTNAME')); end
end
