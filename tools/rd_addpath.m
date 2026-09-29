function rd_addpath()
%RD_ADDPATH  Them cac thu muc du an RobotDelta vao MATLAB path.
%   Chi can khi KHONG mo project RobotDelta.prj. Model RobotDelta_Sim.slx tu goi
%   ham nay (PreLoadFcn) neu chua tim thay RobotDelta_Params tren path.

root = fileparts(fileparts(mfilename('fullpath')));
sub  = {'models','models/import','params','lib','cad/export','cad/solidworks', ...
        'tools','experiments','experiments/tests','experiments/hardware'};
addpath(strjoin(fullfile(root, sub), pathsep));
try
    Simulink.fileGenControl('set','CacheFolder',fullfile(root,'work'), ...
        'CodeGenFolder',fullfile(root,'work'),'createDir',true);
catch
end
end
