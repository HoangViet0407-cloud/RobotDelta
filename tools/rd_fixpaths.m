function rd_fixpaths(mdl)
%RD_FIXPATHS  Bo duong dan tuyet doi trong model de mo duoc tren moi may.
%   rd_fixpaths('RobotDelta')      % model phai dang duoc load; nho save_system sau do
%   - File hinh hoc CAD (File Solid): chi giu ten file, MATLAB tim theo path (cad/export, cad/solidworks).
%     Neu thieu file (vd SolidWorks khong xuat duoc 1 chi tiet), tim file cung ten chi tiet trong cad/.
%   - File du lieu cua Model Workspace: chi giu ten file (tim theo path).
%   Dung sau khi import lai CAD bang smimport tren bat ky may nao.

root = rd_root();
fs = find_system(mdl, 'LookUnderMasks','all', 'RegExp','on', 'ExtGeomFileName','.+');
for k = 1:numel(fs)
    old = get_param(fs{k}, 'ExtGeomFileName');
    [~, nm, ex] = fileparts(old);
    new = [nm ex];
    stp = [dir(fullfile(root, 'cad', '**', [nm '.STEP'])); dir(fullfile(root, 'cad', '**', [nm '.step'])); dir(fullfile(root, 'cad', '**', [nm '.stp']))];
    if ~any(strcmpi(ex, {'.step', '.stp'})) && ~isempty(stp)
        new = stp(1).name;                     % co ban STEP cung ten -> uu tien STEP (doc duoc tren moi may)
    end
    if isempty(dir(fullfile(root, 'cad', '**', new)))
        part = regexprep(nm, '_Default.*$', '');
        c = dir(fullfile(root, 'cad', '**', [part '.*']));
        if ~isempty(c)
            fprintf('  %s: dung %s thay cho %s\n', strrep(fs{k}, newline, ' '), c(1).name, old);
            new = c(1).name;
        else
            warning('RobotDelta:thieuCAD', 'Khong tim thay file CAD "%s" (%s) trong cad/', new, strrep(fs{k}, newline, ' '));
        end
    end
    if ~strcmp(new, old), set_param(fs{k}, 'ExtGeomFileName', new); end
end

ws = get_param(mdl, 'ModelWorkspace');
if any(strcmp(ws.DataSource, {'MATLAB File', 'MAT-File'})) && ~isempty(ws.FileName)
    [~, nm, ex] = fileparts(ws.FileName);
    if ~strcmp(ws.FileName, [nm ex])
        ws.FileName = [nm ex];
        if strcmp(ws.DataSource, 'MATLAB File') && exist([nm ex], 'file'), ws.reload; end
    end
end
end
