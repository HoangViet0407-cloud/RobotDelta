function cfg = E2_DuongTron(Traj, varargin)
%E2_DUONGTRON  Do chinh xac duong di va anh huong cua toc do.
%   Robot ve duong tron len giay (but gan ban dong); chay lan luot 'period' = 4, 2, 1 s
%   de thay sai so / mat buoc tang theo toc do. So vet ve voi duong tron dat va voi mo phong.
%   Tuy chon: 'radius' [m] (0.06) | 'period' [s/vong] (2) | 'vong' so vong (3)

o = inputParser;
o.addParameter('radius', 0.06); o.addParameter('period', 2); o.addParameter('vong', 3);
o.parse(varargin{:}); o = o.Results;

cfg.Traj = Traj;
cfg.Traj.mode = 1; cfg.Traj.radius = o.radius; cfg.Traj.period = o.period;
cfg.T = cfg.Traj.t_ramp + o.vong*o.period;
cfg.mo_ta = sprintf('E2 - Duong tron R = %g mm, %g s/vong (v = %.0f mm/s), %d vong, z = %.1f mm.', ...
    o.radius*1e3, o.period, 2*pi*o.radius/o.period*1e3, o.vong, Traj.home(3)*1e3);
end
