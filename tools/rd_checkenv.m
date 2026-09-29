function ok = rd_checkenv()
%RD_CHECKENV  Kiem tra phien ban MATLAB va cac toolbox can de chay du an.
%   Model luu bang R2025b: may dung ban cu hon khong mo duoc file .slx.

need = ["Simulink", "Simscape", "Simscape Multibody"];
v = ver;
miss = need(~ismember(need, string({v.Name})));
ok = isempty(miss);
if ~ok
    warning('RobotDelta:thieuToolbox', 'May nay thieu: %s -> khong mo phong duoc.', strjoin(miss, ', '));
end
try
    if isMATLABReleaseOlderThan("R2025b")
        ok = false;
        warning('RobotDelta:phienBan', ['Du an luu bang MATLAB R2025b, may nay dung R%s -> khong mo duoc .slx. ' ...
            'Cai R2025b, hoac nho may co R2025b chay: Simulink.exportToVersion(''RobotDelta_Sim'', ''RobotDelta_Sim_cu.slx'', ''R%s'')'], ...
            version('-release'), version('-release'));
    end
catch
end
if ok
    fprintf('MATLAB R%s + Simulink, Simscape, Simscape Multibody: OK\n', version('-release'));
end
end
