clc;
clear all;
close all;

addpath(genpath([getenv('ROBOTPKG_BASE') '/lib/matlab']))


% Add that folder plus all subfolders to the path.
addpath(genpath(pwd));

%% load model
model_path = fullfile(pwd, 'quadrotor_model');
addpath(model_path)

check_acados_requirements();

model = quadrotor_model;
method = 'irk';

%% Simulation parameters
% N = 200; 
N = 20;
% set time horizon 
% T = 2; 
T =  2;
h = T/N;
% Set the initial conditions
x0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0];
% x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0.9227; -0.0191; 0.0462; 0.3822];

%u0 = [596.34; 596.34; 596.34; 596.34]; % 596.34 hovering

% ---------------------- OCP ------------------------ %
[ocp, nx, nu] = init_ocp(x0,T,N);


% ---------------------- SImulation ------------------------ %

[sim] = init_sim(method, h);

% ---------------------- GENOM ------------------------ %
% ROBOT_CHOICE = 'quadrotor';
[robot, rc, opti, pom, maneuver, nhfc] = initialize_genom();

setup_components(robot, rc, opti, pom, maneuver, nhfc);

start(1, rc,  nhfc, maneuver);
% stop(rc,  nhfc, maneuver);
% ---------------------- Closed loop ------------------------ %

%% Simulate system in loop
tsim = 1;
Nsim = N*tsim/T;

x_sim = zeros(nx, Nsim+1);
u_sim = zeros(nu, Nsim);
x_sim(:,1) = x0;


% ---------------------- Traj gen ------------------------ %
% prova ad aggiungere fase iniziale lunga di solo hovering ad altezza zero

% x_end = [2; 0; 0; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0];
x0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0]; %0.986
x_end = [0; 0; 1; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0];
[yref_x] = compute_desired_trajectory(x0(1:3),x_end(1:3), x0(4:6), x_end(4:6), x0(7:9), x_end(7:9), x0(10:13), x_end(10:13), tsim, 0.01);
yref_x(:,1) = x0;
yref_x =[yref_x, repmat(yref_x(:,end), 1, Nsim *100)];
yref_x(4:9, 101:end) = zeros(6, (Nsim *100) + 1 );

% ---------------------- End Traj gen ------------------------ %
% yref_u = 596.34*ones(4, Nsim+1);
yref_u = 69.4949*ones(4, 1101);
yref = vertcat(yref_x,yref_u);
yref = [yref, repmat(yref(:,end),1,20*Nsim)];

yref_e = yref_x;
yref_e =[yref_e, repmat(yref_e(:,end), 1, N)];
yref_e = [yref_e, repmat(yref_e(:,end),1,20*Nsim)];

% ------add initial startup values for mpc in traj---------
yref = [repmat(yref(:,1),1,30) , yref];
yref_e = [repmat(yref_e(:,1),1,30) , yref_e];
% -------------------------------------------------------

mpc(nx, nu, x0, x_sim, u_sim, N, yref, yref_e, ocp, rc, pom, nhfc, maneuver);

% stop(rc,  nhfc, maneuver);

% ----------------------End Closed loop ------------------------ %

% ----------------------Plot Results ------------------------ %


% Plot 3d position as first 3 eleeemnts of state
figure;
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:));
xlabel('x');
ylabel('y');
zlabel('z');
title('3D position');
grid on;

% plot on the same figure the error on position between the reference and the simulated trajectory
figure;
plot(yref(1, 1:501) - x_sim(1, :), 'r');
hold on;
plot(yref(2, 1:501) - x_sim(2, :), 'g');
hold on;
plot(yref(3, 1:501) - x_sim(3, :), 'b');
xlabel('Xref');
ylabel('Yref');
zlabel('Zref');
title('Error on position between the reference and the simulated trajectory');
legend('X', 'Y', 'Z');


%------------------- Real time plot of the quadrotor -------------------------------------

% Create quadrotor body shape
initial_position = [0, 0, 0];
arm_length = 0.2;
initial_orientation = [0, 0, 0];
% Convert Euler angles to rotation matrix
R = eul2rotm(initial_orientation, 'XYZ');
body_vertices = [0, 0, 0;  % Center
                 arm_length/2, 0, 0;  % Front
                 -arm_length/2, 0, 0;  % Back
                 0, arm_length/2, 0;  % Right
                 0, -arm_length/2, 0;  % Left
                 0, 0, arm_length/2;  % Top
                 0, 0, -arm_length/2];  % Bottom

% Transform vertices to world frame using the rotation matrix and initial position
body_vertices_world = (R * body_vertices')' + initial_position;

% Extract the coordinates for plotting
x = body_vertices_world(:, 1);
y = body_vertices_world(:, 2);
z = body_vertices_world(:, 3);



% Create figure and 3D axis
fig = figure;
ax = axes('Parent', fig);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;

% Create the plot
obj = plot3(ax, x, y, z, 'r');  % Plot the body and arms
hold on;

% Plot arms
plot3(ax, [x(1), x(5)], [y(1), y(5)], [z(1), z(5)], 'b');  % Right arm
plot3(ax, [x(1), x(6)], [y(1), y(6)], [z(1), z(6)], 'g');  % Top arm
plot3(ax, [x(1), x(7)], [y(1), y(7)], [z(1), z(7)], 'm');  % Bottom arm

% Customize plot appearance
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');
title(ax, 'Quadrotor Visualization');

% Update object position and orientation for each time step
t_start = 0;  % Start time
t_end = tsim;   % End time
dt = 0.01;    % Time step
tplot = linspace(t_start, t_end, (t_end - t_start) / dt + 1);

for i = 1:length(tplot)
    % Update position
    x = x_sim(1, i);
    y = x_sim(2, i);
    z = x_sim(3, i);
    set(obj, 'XData', x, 'YData', y, 'ZData', z);
    
    % Update orientation (Euler angles)
    eulerAngles = quat2eul(x_sim(10:13,i)', 'XYZ');  % Adjust order as needed
    roll = eulerAngles(1);
    pitch = eulerAngles(2);
    yaw = eulerAngles(3);
    % Update object orientation using rotate function
    rotate(obj, [1, 0, 0], rad2deg(roll), [x, y, z]);  % Rotate around X-axis
    rotate(obj, [0, 1, 0], rad2deg(pitch), [x, y, z]);  % Rotate around Y-axis
    rotate(obj, [0, 0, 1], rad2deg(yaw), [x, y, z]);  % Rotate around Z-axis
    
    % Plot past motion (optional)
    if i > 1
        % Plot line segment connecting previous and current positions
        line(ax, [x_sim(1, i-1), x], [x_sim(2, i-1), y], [x_sim(3, i-1), z], 'Color', 'b');
    end
    % Pause to control animation speed (adjust as needed)
    pause(0.1);  % Adjust pause duration as needed
end
%--------------------------------------------------------------------------



