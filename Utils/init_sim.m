function [sim] = init_sim(method, h)

%% Sim model
model_path = fullfile(pwd, 'quadrotor_model');
addpath(model_path)

check_acados_requirements();

model = quadrotor_model;

ocp_model = acados_ocp_model();
nx = model.nx;
nu = model.nu;
ny = model.ny;


sim_model = acados_sim_model();

sim_model.set('T', h);
sim_model.set('name', 'quadrotor');

sim_model.set('dim_nx', nx);
sim_model.set('dim_nu', nu);
%sim_model.set('dim_np', np);

sim_model.set('sym_x', model.sym_x);
sim_model.set('sym_xdot', model.sym_xdot);
sim_model.set('sym_u', model.sym_u);
%sim_model.set('sym_p', model.sym_p);
sim_model.set('dyn_type', 'implicit');
sim_model.set('dyn_expr_f', model.expr_f_impl);


%% Sim options
sim_opts = acados_sim_opts();

sim_opts.set('compile_interface', 'auto');
sim_opts.set('num_stages', 2);
sim_opts.set('num_steps', 3);
sim_opts.set('newton_iter', 3);
sim_opts.set('method', method);
sim_opts.set('sens_forw', 'true');


%% Create integrator
sim = acados_sim(sim_model, sim_opts);

% ---------------------- End Simulation ------------------------ %









end