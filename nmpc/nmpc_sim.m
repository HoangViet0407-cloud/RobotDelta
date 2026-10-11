function R = nmpc_sim(TrajIn, T, varargin)
%NMPC_SIM  Mo phong vong kin NMPC trong MATLAB (driver ly tuong bam lenh vi tri).
%   R = nmpc_sim()                       quy dao trong RobotDelta_Params, 2 chu ky
%   R = nmpc_sim(Traj, T, 'tau_max', 0.5) doi gioi han mo-men khi chay (khong can build lai)
%   Ket qua: sai so bam TCP, mo-men du bao, thoi gian giai moi chu ky.
%   Doi tuong: x(k+1) = tich phan chinh xac dx = [dq; u] trong Ts (driver vong kin bam lenh).

RobotDelta_Params;                               % G, Dyn, Nmpc, Motor (+ Traj mac dinh)
if nargin >= 1 && ~isempty(TrajIn), Traj = TrajIn; end
if nargin < 2 || isempty(T), T = Traj.t_ramp + 2*Traj.period; end
o = inputParser; o.addParameter('tau_max', Nmpc.tau_max); o.addParameter('solver', []);
o.addParameter('ve', true); o.parse(varargin{:}); o = o.Results;
S = o.solver; if isempty(S), S = RobotDelta_NMPC_build(); end
N = Nmpc.N; Ts = Nmpc.Ts; d2r = pi/180;
for j = 0:N-1
    S.set('constr_lh', -o.tau_max*[1;1;1], j); S.set('constr_uh', o.tau_max*[1;1;1], j);
end

K = round(T/Ts);
x = [RobotDelta_IKd(0, Traj, G)*d2r; 0; 0; 0];
R.t = (0:K-1)'*Ts; R.q = zeros(K,3); R.qr = R.q; R.tau = R.q; R.e_tcp = zeros(K,1);
R.t_giai = zeros(K,1); R.status = zeros(K,1);
S.set('init_x', repmat(x, 1, N+1)); S.set('init_u', zeros(3, N));
for k = 1:K
    t = R.t(k);
    for j = 0:N
        [th, dth, ddth] = RobotDelta_IKd(t + j*Ts, Traj, G);
        if j < N, S.set('cost_y_ref', [th; dth; ddth]*d2r, j);
        else,     S.set('cost_y_ref_e', [th; dth]*d2r); end
    end
    S.set('constr_x0', x);
    S.solve();
    R.status(k) = S.get('status');
    R.t_giai(k) = S.get('time_tot');
    u = S.get('u', 0);
    R.q(k,:) = x(1:3)'/d2r; R.qr(k,:) = RobotDelta_IKd(t, Traj, G)';
    R.tau(k,:) = rd_dyn_sym(x(1:3), x(4:6), u, G, Dyn.phi)' + Motor.J_rotor*Motor.gear^2*u';
    R.e_tcp(k) = norm(reshape(RobotDelta_FK(R.q(k,:)', G),3,1) - reshape(RobotDelta_Traj(t, Traj),3,1));
    x = [x(1:3) + x(4:6)*Ts + u*Ts^2/2; x(4:6) + u*Ts];   % lenh gui xuong driver o chu ky sau
end
fprintf('NMPC (N = %d, Ts = %g ms, tau_max = %.2f N*m): sai so TCP RMS %.3f mm, max %.3f mm | |tau| max %.3f N*m\n', ...
    N, Ts*1e3, o.tau_max, rms(R.e_tcp)*1e3, max(R.e_tcp)*1e3, max(abs(R.tau(:))));
fprintf('  thoi gian giai: trung binh %.3f ms, max %.3f ms (may nay) | loi solver: %d chu ky\n', ...
    mean(R.t_giai)*1e3, max(R.t_giai)*1e3, nnz(R.status ~= 0));
if o.ve
    figure('Name', 'NMPC');
    subplot(3,1,1); plot(R.t, R.qr, '--', R.t, R.q); grid on; ylabel('\theta [deg]'); title('goc tay: dat (--) / NMPC');
    subplot(3,1,2); plot(R.t, R.tau); yline(o.tau_max*[-1 1], 'r:'); grid on; ylabel('\tau [N m]');
    subplot(3,1,3); plot(R.t, R.e_tcp*1e3); grid on; ylabel('sai so TCP [mm]'); xlabel('t [s]');
end
end
