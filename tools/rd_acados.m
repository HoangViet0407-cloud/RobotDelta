function A = rd_acados()
%RD_ACADOS  Nap acados + CasADi vao MATLAB (goi truoc khi tao / chay bo giai NMPC).
%   A = rd_acados()  ->  A.dir, A.casadi, A.cmake
%   Duong dan lay tu local/RobotDelta_Local.m (Local.acadosDir, casadiDir, cmakeDir);
%   de trong thi tu tim o <thu muc chua du an>/tools_ext/{acados,casadi}.
%   Cai dat (1 lan / may): xem README, muc NMPC.

persistent done
L = rd_local();
ext = fullfile(fileparts(rd_root()), 'tools_ext');
A.dir    = pick(L, 'acadosDir', fullfile(ext, 'acados'));
A.casadi = pick(L, 'casadiDir', fullfile(ext, 'casadi'));
A.cmake  = pick(L, 'cmakeDir',  '');
assert(isfile(fullfile(A.dir, 'lib', 'link_libs.json')), ...
    'Chua build acados o %s (xem README, muc NMPC) hoac sua Local.acadosDir', A.dir);
assert(isfolder(fullfile(A.casadi, '+casadi')), 'Khong thay CasADi o %s: sua Local.casadiDir', A.casadi);
if ispc && strlength(A.cmake) == 0 && system('where cmake >nul 2>nul') ~= 0
    c = dir('C:\Program Files*\Microsoft Visual Studio\*\*\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe');
    if ~isempty(c), A.cmake = c(end).folder; end
end
if isempty(done)
    addpath(fullfile(A.dir, 'interfaces', 'acados_matlab_octave'), A.casadi);
    setenv('ACADOS_INSTALL_DIR', A.dir);
    setenv('ENV_RUN', 'true');
    if strlength(A.cmake) > 0, setenv('PATH', [char(A.cmake) pathsep getenv('PATH')]); end
    done = true;
end
end

function v = pick(L, f, def)
v = def;
if isfield(L, f) && strlength(L.(f)) > 0, v = char(L.(f)); end
end
