function p = RobotDelta_Traj(t, Traj)
%#codegen
%ROBOTDELTA_TRAJ  Quy dao TCP [m] (3x1, he toa do robot) tai thoi diem t [s].
%   Dung chung cho mo phong (khoi Quy_Dao) va robot that (exp_run) -> cung mot tham chieu.
%   Traj.mode 0: dung yen tai Traj.home
%             1: duong tron ban kinh Traj.radius quanh Traj.home, 1 vong / Traj.period
%             2: gap-dat A <-> B (Traj.pA, Traj.pB), nhac Traj.lift, 1 chu ky / Traj.period
%             3: di lan luot qua cac diem Traj.wp (Nx3): moi doan Traj.wp_T [s],
%                dung Traj.wp_dwell [s] tai moi diem (bat dau tu Traj.home)

r = min(t/Traj.t_ramp, 1); r = r^3*(10 - 15*r + 6*r^2);  % tang dan 0 -> 1, van toc & gia toc = 0 o 2 dau
switch Traj.mode
    case 1
        w = 2*pi/Traj.period;
        q = Traj.home + r*Traj.radius*[cos(w*t) sin(w*t) 0];
    case 2
        if t < Traj.t_ramp
            q = Traj.home + r*(Traj.pA - Traj.home);
        else
            s = mod(t - Traj.t_ramp, Traj.period)/(Traj.period/2);   % 0..2
            if s < 1, a = Traj.pA; b = Traj.pB; else, a = Traj.pB; b = Traj.pA; s = s - 1; end
            q = a + cyc(s)*(b - a) + [0 0 Traj.lift*64*s^3*(1-s)^3];   % nhac: gia toc = 0 o A, B
        end
    case 3
        n  = size(Traj.wp, 1);
        Ts = Traj.wp_T + Traj.wp_dwell;
        k  = floor(t/Ts);                                    % doan thu k (0 = tu home)
        if k >= n
            q = Traj.wp(n,:);
        else
            if k == 0, a = Traj.home; else, a = Traj.wp(k,:); end
            s = min((t - k*Ts)/Traj.wp_T, 1);
            q = a + cyc(s)*(Traj.wp(k+1,:) - a);
        end
    otherwise
        q = Traj.home;
end
p = q(:);
end

function u = cyc(s)
% cycloid 0 -> 1: van toc va gia toc bang 0 o hai dau
u = s - sin(2*pi*s)/(2*pi);
end
