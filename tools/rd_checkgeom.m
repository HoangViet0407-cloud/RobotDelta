function ok = rd_checkgeom()
%RD_CHECKGEOM  Kiem tra file CAD them tay (cad/solidworks/) co dung phien ban voi cum lap SolidWorks.
%   So trong tam Simscape tinh tu file voi trong tam SolidWorks xuat (smiData, cung he toa do chi tiet).
%   Lech > 1 mm = file khac phien ban / khac cau hinh -> chi tiet se bi ve sai vi tri trong model.

root = rd_root();
RobotDelta_DataFile;                                   % smiData
load_system('sm_lib'); load_system('nesl_utility');
f = dir(fullfile(root, 'cad', 'solidworks', '*.*')); f = f(~[f.isdir]);
ids = string({smiData.Solid.ID});
ok = true;
for k = 1:numel(f)
    [~, part, ex] = fileparts(f(k).name);
    hasStep = any(strcmpi({f.name}, [part '.STEP']) | strcmpi({f.name}, [part '.stp']));
    if ~any(strcmpi(ex, {'.step', '.stp'})) && hasStep, continue; end   % model dung ban STEP
    i = find(startsWith(ids, part + "*"), 1);
    if isempty(i), continue; end
    c = comFromFile(f(k).name, smiData.Solid(i).mass);
    d = c - smiData.Solid(i).CoM;
    fprintf('  %-22s trong tam file %s mm | SolidWorks %s mm | lech %.1f mm\n', f(k).name, ...
        mat2str(round(c, 1)), mat2str(round(smiData.Solid(i).CoM, 1)), norm(d));
    if norm(d) > 1
        ok = false;
        warning('RobotDelta:saiCAD', ['%s KHAC phien ban voi chi tiet trong cum lap (lech %s mm) -> ' ...
            've sai vi tri. Trong SolidWorks: mo dung chi tiet cum lap dang dung, Save As STEP vao cad/solidworks/%s.STEP'], ...
            f(k).name, mat2str(round(d, 1)), part);
    end
end
end

function c = comFromFile(file, mass)
m = 'rd_checkgeom_tmp'; if bdIsLoaded(m), close_system(m, 0); end
new_system(m); P = @(h) get_param(h, 'PortHandles');
w  = add_block('sm_lib/Frames and Transforms/World Frame', [m '/W']);
mc = add_block('sm_lib/Utilities/Mechanism Configuration', [m '/MC']);
sc = add_block('nesl_utility/Solver Configuration', [m '/SC']);
s  = add_block('sm_lib/Body Elements/File Solid', [m '/S'], 'ExtGeomFileName', file, ...
     'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', 'Mass', num2str(mass, 17));
is = add_block('sm_lib/Body Elements/Inertia Sensor', [m '/IS'], 'SenseCenterOfMass', 'on', 'ExcludeGroundedBodies', 'off');
cv = add_block('nesl_utility/PS-Simulink Converter', [m '/C'], 'Unit', 'mm');
o  = add_block('simulink/Sinks/To Workspace', [m '/O'], 'VariableName', 'com', 'SaveFormat', 'Array');
wp = P(w); for h = [s is mc sc], p = P(h); if h == is, add_line(m, wp.RConn(1), p.LConn(1)); else, add_line(m, wp.RConn(1), p.RConn(1)); end, end
ip = P(is); cp = P(cv); op = P(o);
add_line(m, ip.RConn(1), cp.LConn(1)); add_line(m, cp.Outport(1), op.Inport(1));
set_param(m, 'StopTime', '0', 'SimMechanicsOpenEditorOnUpdate', 'off');
out = sim(m);
c = reshape(out.com(1:3), 1, 3);
close_system(m, 0);
end
