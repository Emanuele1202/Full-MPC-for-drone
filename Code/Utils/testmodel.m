function [F, M, a, alpha] = testmodel(state,input)

u = input;
w = [0; 0; 0];

%% System parameters (adjust as needed)
m = 1.00 + 0.07 * 4;   % Mass of the quadrotor [kg]
% m = 1.45;
g = 9.81;  % Gravity constant [m/s^2]
L = 0.23;  % Length from the center of the quadrotor to the motors [m]
theta = deg2rad(0); % Angle between body frame X-axis and leg 1
Ixx = 0.015; % Moment of inertia around x-axis [kg*m^2]
Iyy = 0.015; % Moment of inertia around y-axis [kg*m^2]
Izz = 0.007;  % Moment of inertia around z-axis [kg*m^2]
Ii = diag([Ixx, Iyy, Izz]); % Moment of inertia matrix
ct = 6.5e-4;%/(4 * pi^2);   % Thrust coefficient [N/(rad/s)^2]
cd = 1.0e-5;%/(4 * pi^2);   % Drag coefficient [Nm/(rad/s)^2]
% ct = 1e-5;   % Thrust coefficient [N/(rad/s)^2]
% cd = 1e-7;   % Drag coefficient [Nm/(rad/s)^2]
q = state;

%% Define propeller pos in body frame
p_pos1 = [L*cos(theta); -L*sin(theta); 0];
p_pos2 = [L*sin(theta); L*cos(theta); 0];
p_pos3 = [-L*cos(theta); L*sin(theta); 0];
p_pos4 = [-L*sin(theta); -L*cos(theta); 0];
p_pos = [p_pos1 p_pos2 p_pos3 p_pos4];

% Define the rotation matrix from propeller to body frame
Rpb = eye(3);

% Define the rotation matrix from body to world frame

% Rbw = quat2rotm(q');
Rbw = quat2rot(q);

% Define the thrust force
thrust_p = zeros(3,1);
f_thrust_b = zeros(3,1);

% Define the drag and thrust torque
tau_drag_p = zeros(3,1);
tau_drag_b = zeros(3,1);

tau_thrust_b = zeros(3,1);

for i = 1:4

       thrust_p = sign(u(i))*ct*u(i)^2*Rpb(:,3); % 3x1 vector where each column is referred to the thrust of a propeller along x,y,z
       f_thrust_b = Rpb * thrust_p + f_thrust_b; % 3x1 vector ( sum of the 4 propeller thrust)

       tau_drag_p = (-1)^(i+1) * cd *  sign(u(i)) *u(i)^2 * Rpb(:,3); %  3x1 vector where each column is referred to the drug of a propeller along x,y,z
       tau_drag_b = Rpb * tau_drag_p + tau_drag_b; % 3x1 vector (sum of the 4 propeller drag)

       tau_thrust_b = cross(p_pos(:,i)',(Rpb*thrust_p)')' + tau_thrust_b;

end

% Define matrices for computation
matrix2 = zeros(6,6);
matrix3 = zeros(6,1);
matrix1 = vertcat(-m*[0 0 g]', -cross(w,Ii*w)); %-
matrix2(1:3,1:3) = Rbw;
matrix2(1:3,4:6) = zeros(3,3);
matrix2(4:6,4:6) = eye(3,3);
matrix2(4:6,1:3) = zeros(3,3);
matrix3 = vertcat(f_thrust_b, tau_thrust_b + tau_drag_b);


F = matrix1(1:3) + matrix2(1:3,1:3)*matrix3(1:3);
Fb = Rbw' * F;
M = matrix1(4:6) + matrix2(4:6,4:6)*matrix3(4:6);
 

a = F/m;
alpha = Ii \ M;


end