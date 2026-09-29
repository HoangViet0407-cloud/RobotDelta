function f = rd_package(outDir)
%RD_PACKAGE  Dong goi du an thanh 1 file .mlproj de gui sang may khac (khi khong dung Git).
%   f = rd_package()            % luu canh thu muc du an: RobotDelta_<ngay_gio>.mlproj
%   f = rd_package(thu_muc)
%   Ben nhan: nhap dup file .mlproj trong MATLAB -> chon noi giai nen -> project tu mo.
%   Chi gom file cua du an; KHONG gom local/ (cai dat rieng), work/ (cache), data/ (ket qua).

root = rd_root();
if nargin < 1, outDir = fileparts(root); end
proj = openProject(root);
for d = ["models", "params", "lib", "cad", "tools", "experiments"]
    addFolderIncludingChildFiles(proj, fullfile(root, d));      % gom ca file moi them
end
for x = [".gitignore", ".gitattributes", "README.md"]
    if isfile(fullfile(root, x)), addFile(proj, fullfile(root, x)); end
end
f = fullfile(outDir, "RobotDelta_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmm')) + ".mlproj");
% bo phan tich phu thuoc bao nham: bien tao boi script (G, Traj...) va ham cua serialport
w = warning('off', 'MATLAB:project:api:ExportWithMissingDependencies');
export(proj, f);
warning(w);
s = dir(f);
fprintf('Da dong goi: %s (%.1f MB)\n', f, s.bytes/1e6);
end
