function cfg = E3_GapDat(Traj, varargin)
%E3_GAPDAT  Chu ky gap - dat A <-> B voi giac hut.
%   Do: thoi gian chu ky, ti le gap thanh cong, co mat buoc khong (robot ve dung home sau N chu ky?).
%   Giam dan 'period' den khi mat buoc / roi vat -> toc do thuc te toi da; so mo-men voi mo phong.
%   Bom hut (io = 1) bat khi di A -> B, tat khi quay ve.
%   Tuy chon: 'khoang' [m] A-B (0.12) | 'lift' [m] (0.04) | 'period' [s] (2) | 'chu_ky' (5)

o = inputParser;
o.addParameter('khoang', 0.12); o.addParameter('lift', 0.04);
o.addParameter('period', 2); o.addParameter('chu_ky', 5);
o.parse(varargin{:}); o = o.Results;

cfg.Traj = Traj;
cfg.Traj.mode = 2;
cfg.Traj.pA = Traj.home + [-o.khoang/2 0 0];
cfg.Traj.pB = Traj.home + [ o.khoang/2 0 0];
cfg.Traj.lift = o.lift; cfg.Traj.period = o.period;
cfg.T = cfg.Traj.t_ramp + o.chu_ky*o.period;
tr = cfg.Traj.t_ramp; per = o.period;
cfg.io = @(t) double(t >= tr & mod(t - tr, per) < per/2);
cfg.mo_ta = sprintf('E3 - Gap-dat: A-B %g mm, nhac %g mm, %g s/chu ky, %d chu ky.', ...
    o.khoang*1e3, o.lift*1e3, o.period, o.chu_ky);
end
