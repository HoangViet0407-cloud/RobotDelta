function J = exp_jointtraj(Traj, T, G, Lim, Motor, dt, verbose)
%EXP_JOINTTRAJ  Quy dao TCP -> quy dao goc dong co + kiem tra gioi han (dung chung sim & robot that).
%   J = exp_jointtraj(Traj, T, G, Lim, Motor)            % dt = 0.01 s
%   J.t [s], J.xyz [m], J.theta [deg], J.omega [deg/s], J.alpha [deg/s^2]
%   J.tilt   [deg] goc nghieng bi mat trau,   J.gamma [deg] goc tay - thanh truyen
%   J.pulse_hz     tan so xung buoc can thiet cho moi dong co
%   J.issues       danh sach vi pham (rong = dat),  J.pass

if nargin < 6 || isempty(dt), dt = 0.01; end
if nargin < 7, verbose = true; end

J.t = (0:dt:T)';
n = numel(J.t);
J.xyz = zeros(n,3); J.theta = zeros(n,3); reach = true(n,3);
for k = 1:n
    J.xyz(k,:) = RobotDelta_Traj(J.t(k), Traj)';
    [th, ok] = RobotDelta_IK(J.xyz(k,:), G);
    J.theta(k,:) = th'; reach(k,:) = ok';
end
J.omega = zeros(n,3); J.alpha = zeros(n,3);
for i = 1:3
    J.omega(:,i) = gradient(J.theta(:,i), dt);
    J.alpha(:,i) = gradient(J.omega(:,i), dt);
end
M = RobotDelta_Metrics(J.xyz, J.theta, G);
J.tilt = M.tilt; J.gamma = M.gamma;
stepDeg = 360/(Motor.steps_rev*Motor.microstep*Motor.gear);
J.pulse_hz = abs(J.omega)/stepDeg;

S.theta_min = min(J.theta,[],1);   S.theta_max = max(J.theta,[],1);
S.omega_max = max(abs(J.omega),[],1); S.alpha_max = max(abs(J.alpha),[],1);
S.pulse_max = max(J.pulse_hz,[],1);   S.tilt_max  = max(J.tilt,[],1);
S.gamma_min = min(J.gamma,[],1);      S.gamma_max = max(J.gamma,[],1);
J.summary = S;

iss = {};
if ~all(reach(:))
    iss{end+1} = sprintf('%d diem ngoai tam voi (IK bao hoa)', nnz(any(~reach,2)));
end
if any(S.theta_min < Lim.theta(1) | S.theta_max > Lim.theta(2))
    iss{end+1} = sprintf('goc tay ra ngoai [%g, %g] deg', Lim.theta);
end
if any(S.tilt_max > Lim.tilt_rodend)
    iss{end+1} = sprintf('bi mat trau nghieng %.1f deg > %g deg', max(S.tilt_max), Lim.tilt_rodend);
end
if any(S.gamma_min < Lim.gamma(1) | S.gamma_max > Lim.gamma(2))
    iss{end+1} = sprintf('goc tay-thanh truyen ra ngoai [%g, %g] deg (gan ky di)', Lim.gamma);
end
if ~isnan(Lim.omega) && any(S.omega_max > Lim.omega)
    iss{end+1} = sprintf('toc do %.0f deg/s > %g deg/s', max(S.omega_max), Lim.omega);
end
J.issues = iss;
J.pass = isempty(iss);

if verbose
    fprintf('  %-28s %9s %9s %9s\n', '', 'Chan 1', 'Chan 2', 'Chan 3');
    row = @(name, v, fmt) fprintf(['  %-28s' repmat([' ' fmt], 1, 3) '\n'], name, v);
    row('goc tay min / max [deg]',   [S.theta_min; S.theta_max], '%4.0f/%-4.0f');
    row('toc do max [deg/s]',        S.omega_max, '%9.0f');
    row('gia toc max [deg/s^2]',     S.alpha_max, '%9.0f');
    row('xung buoc max [Hz]',        S.pulse_max, '%9.0f');
    row('nghieng bi mat trau [deg]', S.tilt_max,  '%9.1f');
    row('goc tay-thanh min/max [deg]', [S.gamma_min; S.gamma_max], '%4.0f/%-4.0f');
    if J.pass
        fprintf('  => DAT: khong vi pham gioi han\n');
    else
        fprintf('  => VI PHAM: %s\n', strjoin(iss, '; '));
    end
end
end
