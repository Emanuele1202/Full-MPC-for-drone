function [reference_states] = compute_desired_trajectory(pos_init, pos_end, vel_lin_init, vel_lin_end, vel_ang_init, vel_ang_end, q_start, q_end, duration, sample_time)
    
    % Generate polynomial trajectory
    t = 0:sample_time:duration;
    x_poly = polyfit([0, duration], [pos_init(1), pos_end(1)], 5);
    y_poly = polyfit([0, duration], [pos_init(2), pos_end(2)] ,5);
    z_poly = polyfit([0, duration], [pos_init(3), pos_end(3)], 5);
    x_trajectory = polyval(x_poly, t);
    y_trajectory = polyval(y_poly, t);
    z_trajectory = polyval(z_poly, t);
    
    % Get reference state for each time instant
    desired_pos = [x_trajectory', y_trajectory', z_trajectory']';

    % Calculate desired velocity and acceleration
    dx_dt = polyder(x_poly);
    dy_dt = polyder(y_poly);
    dz_dt = polyder(z_poly);
    desired_lin_velocity = [polyval(dx_dt, t)', polyval(dy_dt, t)', polyval(dz_dt, t)']';
    
    %d2x_dt2 = polyder(dx_dt);
    %d2y_dt2 = polyder(dy_dt);
    %d2z_dt2 = polyder(dz_dt);
    %desired_acceleration = [polyval(d2x_dt2, t)', polyval(d2y_dt2, t)', polyval(d2z_dt2, t)']';

    % Interpolate quaternions between start and end orientations
    q_interp = slerp_quaternions(q_start, q_end, t);

    % Compute Angular velocity as the derivative of the interpolated quaternions 
    % Compute angular velocity
    q_dot = zeros(3, 101); %Nsim = 500
    % for i = 1:(100-1)
    %     % Quaternion derivativ
    %     q_dot(:, i) = (q_interp(:, i+1) - q_interp(:, i)) / sample_time;
    % end
    % Last element
    %q_dot(:, end) = q_dot(:, end-1);  % Use backward difference approximation for the last point
    
    

    % Combine all desired states into a single matrix
    %reference_states is a matrix with 13 rows and t columns
    reference_states = zeros(13, length(t));

    reference_states(1:3, :) = desired_pos;
    reference_states(4:6, :) = desired_lin_velocity;
    reference_states(7:9, :) = q_dot;
    reference_states(10:13, :) = q_interp;

    
    % Plot the trajectory in 3D 
    figure;
    plot3(desired_pos(1, :), desired_pos(2, :), desired_pos(3, :), 'b', 'LineWidth', 2);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Desired Trajectory');
    grid on;
    

end

%% Define q_interp function
function q_interp_quaternions = slerp_quaternions(q_start, q_end, t)
    % Spherical linear interpolation of quaternions
    dot_product = dot(q_start', q_end');
    if dot_product < 0
        q_end = -q_end;
        dot_product = -dot_product;
    end
    theta = acos(dot_product);
    
    % Initialize the output matrix
    q_interp_quaternions = zeros(4, length(t));
    
    % Compute the interpolated quaternion for each time instance
    for i = 1:length(t)
        % Compute scalar terms
        term1 = sin((1 - t(i)) * theta);
        term2 = sin(t(i) * theta);
        
        % Compute interpolated quaternion for time instance i
        % Handle singularity when theta is close to zero
        if abs(theta) < eps
            q_interp_quaternions(:, i) = q_start;
        else
            % Compute interpolated quaternion for time instance i
            q_interp_quaternions(:, i) = (term1 * q_start + term2 * q_end) / sin(theta);
        end 
    end
end
