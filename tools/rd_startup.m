function rd_startup()
%RD_STARTUP  Tu chay khi mo project RobotDelta (moi may, moi lan mo).
%   Tao thu muc work/, data/ neu thieu; kiem tra MATLAB + toolbox; tao cai dat rieng cua may.

root = rd_root();
for d = ["work", "data"]
    if ~isfolder(fullfile(root, d)), mkdir(fullfile(root, d)); end
end
fprintf('\n=== Du an RobotDelta: %s ===\n', root);
rd_checkenv();
L = rd_local();
fprintf('May: %s | nguoi dung: %s | cong Serial: %s\nKet qua luu tai: %s\n', L.may, L.user, L.port, L.dataDir);
fprintf('Bat dau: mo "RobotDelta_Sim" (Shortcuts) | exp_precheck | exp_run(''E2_DuongTron'',''sim'')\n\n');
end
