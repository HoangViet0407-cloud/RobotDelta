function solver = RobotDelta_NMPC_build(simulink, tau_max)
%ROBOTDELTA_NMPC_BUILD  Tao bo giai NMPC bang acados (sinh ma C + bien dich).
%   solver = RobotDelta_NMPC_build()        -> AcadosOcpSolver dung trong MATLAB
%   RobotDelta_NMPC_build(true)             -> them S-function cho Simulink
%                                              (work/acados_solver_sfunction_robotdelta.mexw64)
%   RobotDelta_NMPC_build(true, 0.8)        -> gioi han mo-men khac Nmpc.tau_max (thu nghiem)
%
%   Bai toan (phuong an A: driver buoc vong kin nhan lenh vi tri):
%     trang thai x = [q; dq] (3 goc tay [rad] + van toc),  dau vao u = ddq [rad/s^2]
%     mo hinh     dx = [dq; u]   (driver bam lenh vi tri)
%     muc tieu    sum ||q - q_r||^2_Wq + ||dq - dq_r||^2_Wdq + ||u - ddq_r||^2_Wu
%     rang buoc   goc tay, van toc, gia toc trong gioi han;
%                 |tau(q,dq,u)| <= tau_max  voi tau tu mo hinh dong luc hoc rd_dyn_sym
%                 (+ quan tinh roto quy doi), rang buoc mem de luon co nghiem.
%   Ma C sinh ra o work/nmpc_c (khong dua len Git); build lai tren Raspberry Pi tu cung bai toan.

if nargin < 1, simulink = false; end
rd_acados();
import casadi.*
RobotDelta_Params;
P = Nmpc;
if nargin >= 2 && ~isempty(tau_max), P.tau_max = tau_max; end

q = SX.sym('q', 3); dq = SX.sym('dq', 3); u = SX.sym('u', 3);
tau = rd_dyn_sym(q, dq, u, G, Dyn.phi) + Motor.J_rotor*Motor.gear^2*u;

model = AcadosModel();
model.name = 'robotdelta';
model.x = [q; dq];
model.u = u;
model.f_expl_expr = [dq; u];
model.con_h_expr   = tau;
model.con_h_expr_0 = tau;

ocp = AcadosOcp();
ocp.model = model;
ocp.name = 'robotdelta';                       % ten co dinh -> S-function acados_solver_sfunction_robotdelta
ocp.code_export_directory = fullfile(rd_root(), 'work', 'nmpc_c');
ocp.json_file = fullfile(rd_root(), 'work', 'robotdelta_ocp.json');

% chi phi LINEAR_LS: y = [x; u]
ocp.cost.cost_type_0 = 'LINEAR_LS'; ocp.cost.cost_type = 'LINEAR_LS'; ocp.cost.cost_type_e = 'LINEAR_LS';
W = diag([P.Wq*[1 1 1], P.Wdq*[1 1 1], P.Wu*[1 1 1]]);
Vx = [eye(6); zeros(3,6)]; Vu = [zeros(6,3); eye(3)];
ocp.cost.Vx_0 = Vx; ocp.cost.Vu_0 = Vu; ocp.cost.W_0 = W; ocp.cost.yref_0 = zeros(9,1);
ocp.cost.Vx = Vx;   ocp.cost.Vu = Vu;   ocp.cost.W = W;     ocp.cost.yref = zeros(9,1);
ocp.cost.Vx_e = eye(6); ocp.cost.W_e = W(1:6,1:6); ocp.cost.yref_e = zeros(6,1);

% rang buoc
d2r = pi/180;
ocp.constraints.x0 = [G.th0(:)*d2r; 0; 0; 0];
ocp.constraints.idxbx = 0:5;
ocp.constraints.lbx = [Lim.theta(1)*[1 1 1], -P.omega_max*[1 1 1]]'*d2r;
ocp.constraints.ubx = [Lim.theta(2)*[1 1 1],  P.omega_max*[1 1 1]]'*d2r;
ocp.constraints.idxbu = 0:2;
ocp.constraints.lbu = -P.acc_max*d2r*[1 1 1]';
ocp.constraints.ubu =  P.acc_max*d2r*[1 1 1]';
ocp.constraints.lh = -P.tau_max*[1 1 1]'; ocp.constraints.uh = P.tau_max*[1 1 1]';
ocp.constraints.lh_0 = ocp.constraints.lh; ocp.constraints.uh_0 = ocp.constraints.uh;
% mem hoa rang buoc mo-men (phat lon khi vuot) -> QP luon kha thi
ocp.constraints.idxsh = 0:2; ocp.constraints.idxsh_0 = 0:2;
z = 1e6*[1 1 1]'; Z = 1e4*[1 1 1]';   % phat L1 du lon (> nhan tu Lagrange) -> giu dung gioi han khi kha thi
ocp.cost.zl = z; ocp.cost.zu = z; ocp.cost.Zl = Z; ocp.cost.Zu = Z;
ocp.cost.zl_0 = z; ocp.cost.zu_0 = z; ocp.cost.Zl_0 = Z; ocp.cost.Zu_0 = Z;

o = ocp.solver_options;
o.N_horizon = P.N;
o.tf = P.N*P.Ts;
o.nlp_solver_type = 'SQP_RTI';                  % 1 vong SQP / chu ky: thoi gian giai co dinh
o.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
o.qp_solver_cond_N = 5;
o.hessian_approx = 'GAUSS_NEWTON';
o.integrator_type = 'ERK';
o.sim_method_num_stages = 4;
o.sim_method_num_steps = 1;

if simulink
    % cong vao: x0 (lbx_0 = ubx_0), tham chieu cac buoc | cong ra: u0, x1, trang thai, thoi gian giai
    % chu ky mau = buoc dau cua tam du bao = Nmpc.Ts
    so = AcadosOcpSimulinkOptions();
    for f = string(properties(so.inputs))',  if isnumeric(so.inputs.(f)),  so.inputs.(f) = 0;  end, end
    for f = string(properties(so.outputs))', if isnumeric(so.outputs.(f)), so.outputs.(f) = 0; end, end
    so.inputs.lbx_0 = 1; so.inputs.ubx_0 = 1;
    so.inputs.y_ref_0 = 1; so.inputs.y_ref = 1; so.inputs.y_ref_e = 1;
    so.outputs.u0 = 1; so.outputs.x1 = 1; so.outputs.solver_status = 1; so.outputs.CPU_time = 1;
    ocp.simulink_opts = so;
end

here = pwd; cleanup = onCleanup(@() cd(here));
cd(fullfile(rd_root(), 'work'));               % acados de file tam o thu muc hien hanh
solver = AcadosOcpSolver(ocp);
if simulink
    cd(ocp.code_export_directory);
    make_sfun;
    movefile(['acados_solver_sfunction_' ocp.name '.' mexext], fullfile(rd_root(), 'work'), 'f');
end
end
