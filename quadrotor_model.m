 function model = quadrotor_model()

import casadi.*
    
%% System dimensions
nx = 13; % Number of states (position, velocity, quaternions)
nu = 4;  % Number of control inputs (w1, w2, w3, w4)
np = 21; % Number of parameters (reference)
nq = 4; % Number of quaternions
ny = nx + nu; % Number of states + control inputs
ny_e = nx; % Number of states at the endpoint
    
    
%% System parameters (adjust as needed)
m = 1.00 + 0.07 * 4;   % Mass of the quadrotor [kg]
g = 9.81;  % Gravity constant [m/s^2]
L = 0.23;  % Length from the center of the quadrotor to the motors [m]
theta = deg2rad(0); % Angle between body frame X-axis and leg 1
Ixx = 0.015; % Moment of inertia around x-axis [kg*m^2]
Iyy = 0.015; % Moment of inertia around y-axis [kg*m^2]
Izz = 0.007;  % Moment of inertia around z-axis [kg*m^2]
Ii = diag([Ixx, Iyy, Izz]); % Moment of inertia matrix
ct = 6.5e-4;   % Thrust coefficient [N/(rad/s)^2]
cd = 1.0e-5;   % Drag coefficient [Nm/(rad/s)^2]

%% Define propeller pos in body frame
p_pos1 = [L*cos(theta); -L*sin(theta); 0];
p_pos2 = [L*sin(theta); L*cos(theta); 0];
p_pos3 = [-L*cos(theta); L*sin(theta); 0];
p_pos4 = [-L*sin(theta); -L*cos(theta); 0];
p_pos = [p_pos1 p_pos2 p_pos3 p_pos4];
    
%% System dynamics
    
    % Extract states
px = SX.sym('px');   % Position x [m]
py = SX.sym('py');   % Position y [m]
pz = SX.sym('pz');   % Position z [m]
% Define position vector
p = [px py pz]';

vx = SX.sym('vx');   % Vel lin x [m/s]
vy = SX.sym('vy');   % Vel lin y [m/s]
vz = SX.sym('vz');   % Vel lin z [m/s]
% Define velocity vector
v = [vx vy vz]';

wx = SX.sym('wx');   % Vel ang x
wy = SX.sym('wy');   % Vel ang y
wz = SX.sym('wz');   % Vel ang z
% Define angular velocity vector
w = [wx wy wz]';

qw = SX.sym('qw');   % Quaternion scalar part
qx = SX.sym('qx');   % Quaternion x-component
qy = SX.sym('qy');   % Quaternion y-component
qz = SX.sym('qz');   % Quaternion z-component
% Define quaternion vector
q = [qw qx qy qz]';
    
% Extract control inputs
w1 = SX.sym('w1');   % Angular velocity motor 1 [rad/s]
w2 = SX.sym('w2');   % Angular velocity motor 2 [rad/s]
w3 = SX.sym('w3');   % Angular velocity motor 3 [rad/s]
w4 = SX.sym('w4');   % Angular velocity motor 4 [rad/s]
% Define control input vector
u = [w1 w2 w3 w4]';
    
%% Named symbolic variables
sym_x = vertcat(p, v, w, q);

sym_u = u;
    
% Named symbolic variables for the state derivatives
ax = SX.sym('ax');   % Acc lin x [m/s^2]
ay = SX.sym('ay');   % Acc lin y [m/s^2]
az = SX.sym('az');   % Acc lin z [m/s^2]
% Define linear acceleration vector
a = [ax ay az]';

alphax = SX.sym('alpha');   % Angular acceleration x
alphay = SX.sym('alpha');   % Angular acceleration y
alphaz = SX.sym('alpha');   % Angular acceleration z
% Define angular acceleration vector
alpha = [alphax alphay alphaz]';

qdotx = SX.sym('qdotx');   % Quaternion derivative x
qdoty = SX.sym('qdoty');   % Quaternion derivative y
qdotz = SX.sym('qdotz');   % Quaternion derivative z
qdotw = SX.sym('qdotw');   % Quaternion derivative w
% Define quaternion derivative vector
qdot = [qdotx qdoty qdotz qdotw]';

%sym_xdot = vertcat(v, a, alpha , qdot);
sym_xdot = SX.sym('sym_xdot', nx, 1);

    
% -------------------------- System dynamics ------------------------------
% Define F/M vector
F = SX.sym('F', 3);
M = SX.sym('M', 3);

% Define the rotation matrix from propeller to body frame
Rpb = eye(3);

% Define the rotation matrix from body to world frame
% Rbw = [q(1)^2 + q(2)^2, q(2)*q(3)-q(1)*q(4), q(2)*q(4)+q(1)*q(3);
%        q(2)*q(3)+q(1)*q(4), q(1)^2-1+q(3)^2, q(3)*q(4)-q(1)*q(2);
%        q(2)*q(4)-q(1)*q(3), q(3)*q(4)+q(1)*q(2), q(1)^2-1+2*q(4)^2];
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
matrix2 = SX.sym('matrix2', 6, 6);
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
Mw = Rbw * M;
 

a = F/m;
alpha = Ii \ M;

% ---------------------------------------------------------------------------    

% Compute angular velocity
kron_v =     [0, -wx, -wy, -wz;
              wx, 0, wz, -wy;
              wy, -wz, 0, wx;
              wz, wy, -wx, 0];
qdot =  0.5 * kron_v * q;
    
%%  Derivative state
expr_f_expl = vertcat(vx, ...
                          vy, ...
                          vz, ...
                          a(1), ...
                          a(2), ...
                          a(3), ...
                          alpha(1), ...
                          alpha(2), ...
                          alpha(3), ...
                          qdot(1), ...
                          qdot(2), ...
                          qdot(3), ...
                          qdot(4));
    
expr_f_impl = expr_f_expl - sym_xdot; 
     
% Add constraint on the norm of the quaternion (unit quaternion)
q_norm_constraint = 1 - sqrt(qw^2 + qx^2 + qy^2 + qz^2);
% Costraint on the input and state
expr_h = vertcat(sym_x, sym_u); %q_norm_constraint;
% Endpoint constraint
expr_h_e = vertcat(sym_x);

% Cost
expr_y = vertcat(sym_x, sym_u);
expr_y_e = sym_x;


% Initialize desired cost value to zero
    


%% Populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.nq = nq;
model.ny = ny;
model.ny_e = ny_e;
model.sym_x = sym_x;
model.sym_u = sym_u;
model.sym_xdot = sym_xdot;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_h_e = expr_h_e;
model.expr_y = expr_y;
model.expr_y_e = expr_y_e;

%-------- CHECK------%
model.a = a;
model.alpha = alpha;
model.v = v;
model.qdot = qdot;


end
