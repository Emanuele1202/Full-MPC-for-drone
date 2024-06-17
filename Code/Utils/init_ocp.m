function [ocp, nx, nu] = init_ocp(x0,T,N)

%% test of native matlab interface
if ~exist('simulink_opts') %true if it does not exist in this directory
    disp('using acados simulink default options') 
    simulink_opts = get_acados_simulink_opts; 
end

%% OCP Model
model_path = fullfile(pwd, 'quadrotor_model');
addpath(model_path)

check_acados_requirements();

model = quadrotor_model;

ocp_model = acados_ocp_model();

nx = model.nx;
nu = model.nu;
ny = model.ny;

ocp_model.set('name', 'quadrotor');
ocp_model.set('T', T);

% Define NLP solver
nlp_solver = 'sqp';
qp_solver = 'partial_condensing_hpipm'; 
qp_solver_cond_N = 5; 

% Set dimensions
ocp_model.set('dim_nx', nx);
ocp_model.set('dim_nu', nu);
ocp_model.set('dim_ny', nx+nu);
ocp_model.set('dim_ny_e', nx);

% cost
cost_y_ref = zeros(ny, 1);
cost_y_ref_e = zeros(nx, 1);

ocp_model.set('cost_type', 'nonlinear_ls');
ocp_model.set('cost_type_e', 'nonlinear_ls');

ocp_model.set('cost_expr_y', model.expr_y);
ocp_model.set('cost_expr_y_e', model.expr_y_e);

ocp_model.set('cost_y_ref', cost_y_ref);
ocp_model.set('cost_y_ref_e', cost_y_ref_e);



W_x = diag([1e1, 1e1, 1e1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
W_u = diag([1e-2, 1e-2, 1e-2, 1e-2]);
W = blkdiag(W_x, W_u);
W_e = W_x*10e-2;

ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);

% dynamics
ocp_model.set('dyn_type', 'implicit');
ocp_model.set('dyn_expr_f', model.expr_f_impl);
% ocp_model.set('dyn_type', 'explicit');
% ocp_model.set('dyn_expr_f', model.expr_f_expl);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% constraints
ocp_model.set('constr_x0', x0);
ocp_model.set('constr_lbx_0', x0); 

p_limit = 10000*ones(3,1);
v_limit = 2000*ones(3,1);
w_limit = 2000*ones(3,1);
q_limit = 10000*ones(4,1);
limit_lh_e = [-p_limit; -v_limit; -w_limit; -q_limit];
limit_uh_e = [p_limit; v_limit; w_limit; q_limit];
u_limit_up = 1000*ones(4,1);
u_limit_down = 0*ones(4,1);
limit_lh = [-p_limit; -v_limit; -w_limit; -q_limit; u_limit_down];
limit_uh = [p_limit; v_limit; w_limit; q_limit; u_limit_up];

ocp_model.set('constr_lh', limit_lh);
ocp_model.set('constr_uh', limit_uh);
ocp_model.set('constr_lh_e', limit_lh_e);
ocp_model.set('constr_uh_e', limit_uh_e);

ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_expr_h_e', model.expr_h_e);

%% OCP Options
ocp_opts = acados_ocp_opts();

ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('param_scheme_N',N);
%ocp_opts.set('sim_method', method);
%ocp_opts.set('regularize_method', 'project');
ocp_opts.set('nlp_solver_max_iter', 1);
ocp_opts.set('print_level', 0);

ocp = acados_ocp(ocp_model, ocp_opts, simulink_opts);


% ---------------------- End OCP ------------------------ %




end