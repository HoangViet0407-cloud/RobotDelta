function cfg = E1_DoChinhXac(Traj, varargin)
%E1_DOCHINHXAC  Do chinh xac (AP) va do lap lai (RP) vi tri, theo tinh than ISO 9283.
%   5 diem tren mat phang z = Traj.home(3): tam + 4 dinh hinh vuong canh 2a, lap N chu ky.
%   Tai moi diem robot ha but cham giay roi nhac len h -> de lai 1 vet.
%   Sau khi chay: do toa do cac vet (goc = vet tam, truc x theo huong +x cua robot),
%   dien vao do_tay.csv trong thu muc ket qua, roi chay exp_analyze('<thu muc>') -> AP, RP.
%   Tuy chon: 'a' [m] (0.04) | 'N' so chu ky (10) | 'h' [m] do nhac (0.01) | 'T' [s/doan] (0.8)

o = inputParser;
o.addParameter('a', 0.04); o.addParameter('N', 10); o.addParameter('h', 0.01); o.addParameter('T', 0.8);
o.parse(varargin{:}); o = o.Results;

P  = Traj.home + o.a*[0 0 0; 1 1 0; -1 1 0; -1 -1 0; 1 -1 0];
up = [0 0 o.h];
wp = zeros(0,3); id = zeros(0,1);
for n = 1:o.N
    for j = 1:5
        wp = [wp; P(j,:)+up; P(j,:); P(j,:)+up]; %#ok<AGROW>
        id(end+1,1) = j; %#ok<AGROW>
    end
end
cfg.Traj = Traj;
cfg.Traj.mode = 3; cfg.Traj.wp = wp; cfg.Traj.wp_T = o.T; cfg.Traj.wp_dwell = 0.3;
cfg.T = size(wp,1)*(o.T + 0.3) + 0.5;
cfg.diem_do.id = id;
cfg.diem_do.xyz = P(id,:);
cfg.mo_ta = sprintf(['E1 - Do chinh xac & lap lai: 5 diem (tam + hinh vuong %g mm), %d chu ky, ' ...
    'but cham giay tai z = %.1f mm.\n     Do cac vet -> dien do_tay.csv -> exp_analyze(thu muc) de tinh AP/RP.'], ...
    2*o.a*1e3, o.N, Traj.home(3)*1e3);
end
