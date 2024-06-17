function [] = mpc(nx, nu, x0, x_sim, u_sim, N, yref, yref_e, ocp, rc, pom, nhfc, maneuver)
var = true;
rc.start();

ii = 1;
time_inst = 0.01;
u_given = zeros(4, length(yref));
pos_history = zeros(13, length(yref));
dT = 1/200;
x_traj_init = zeros(13, N+1);
u_traj_init = zeros(4, N);

input("start?")
% vel = {69.4949 ,69.4949, 69.4949, 69.4949, 0, 0, 0, 0};
% rc.set_velocity(vel);
while var
    tstart = tic;
    %maneuver.set_current_state();
    state = pom.frame('robot').frame;
    
    
    %x0 = x_sim(:,ii);
    q = [state.att.qw, state.att.qx, state.att.qy, state.att.qz];
    w = [state.avel.wx; state.avel.wy; state.avel.wz];
    Rbw = quat2rotm(q);
    w_b = Rbw'*w;
    x0 = [state.pos.x; state.pos.y; state.pos.z;
          state.vel.vx; state.vel.vy; state.vel.vz;
          w_b(1); w_b(2); w_b(3);
          state.att.qw; state.att.qx; state.att.qy; state.att.qz];
    pos_history(:, ii) = x0;
    ocp.set('constr_x0', x0);
    %initialize cost_y_ref for N steps with the reference trajectory at each step
     for j=0:N-1
         yref_loop = yref(:, ii+j);
         ocp.set('cost_y_ref', yref_loop, j); % Problem
     end
    % ocp.set('cost_y_ref', yref(:, ii));
    ocp.set('cost_y_ref_e', yref_e(:, ii+N)); 

    ocp.set('init_x', x_traj_init);
    ocp.set('init_u', u_traj_init);

    ocp.solve();

    if 1
		status = ocp.get('status');
        ocp.print('stat', 2)
        J = ocp.get_cost();
        J_value(ii) = J;
    end

    % Get solution 
    u0 = ocp.get('u', 0);
    x0 = ocp.get('x', 0);

    x_traj_init = ocp.get('x');
    u_traj_init = ocp.get('u');

    x_traj_init = [x_traj_init(:,2:end), x_traj_init(:,end)];
	u_traj_init = [u_traj_init(:,2:end), u_traj_init(:,end)];
    
    u_given(:, ii) = u0;
    
    % Print solution
    xprint = zeros(nx, N);
    uprint = zeros(nu, N);
    for j=0:N-1
        xprint(:, j+1) = ocp.get('x', j);
        uprint(:, j+1) = ocp.get('u', j);
        for i = 1:13
            if abs(xprint(i,j+1)) < 1e-6
                xprint(i,j+1) = 0;
            end
        end
    end
    % make plot xprint(1:3,:) to see the trajectory
    % plot_pos_function(xprint, 'Predicted Traj');  % use to debug
    % plot(xprint(3, :));

    % Set vel
    vel = {u0(1) , u0(2), u0(3), u0(4), 0, 0, 0, 0};
    if ii > 20
        nhfc.stop();
        % maneuver.stop();
        rc.set_velocity('-a', vel);

    end
    
    % update x0 with actual pos for next iteration
    telapsed = toc(tstart);
    telapsed_values(ii) = telapsed;

    tot_time = (telapsed)/time_inst;
    % pause(max(0, dT - toc));
    % ii = ii + 1 + round(max(0,telapsed - dT));
    ii = round(ii+tot_time);
    

    if ii >= 1000
        var = false;
        % j = 1;
        % while j < 1000
        %     if pos_history(1:3, j+1) == zeros(3,1)
        %         pos_history(1:3, j+2);
        %     end
        %     j= j+1;
        % end
        j = 1;
        position = pos_history;
while j < size(pos_history, 2) - 1
    % Check if the current position is zero
    if all(pos_history(1:3, j) == zeros(3,1))
        % Find the first non-zero next value
        next_non_zero_index = find(any(pos_history(:, j+1:end) ~= 0, 1), 1, 'first');
        
        % Replace the zero values with the first non-zero next value
        if ~isempty(next_non_zero_index)
            pos_history(1:3, j) = pos_history(1:3, j+next_non_zero_index);
        else
            % If there are no non-zero values after the current position,
            % you may choose to break the loop or handle the case as needed.
            % Here, we break the loop.
            break;
        end
    end
    j = j + 1;
end

        
        figure;
        plot(telapsed_values, 'LineWidth', 1.5);
        xlabel('Iteration');
        ylabel('Time (seconds)');
        title('Elapsed Time at Each Iteration');
        grid on;

        figure;
        plot(J_value, 'LineWidth', 1.5);
        xlabel('Iteration');
        ylabel('Cost');
        yscale log;
        title('Cost at Each Iteration');
        grid on;
    
        figure;
        plot(position(1, :) - yref(1,:), LineWidth=2);
        hold on;
        plot(position(2, :) - yref(2,:), LineWidth=2);
        hold on;
        plot(position(3, :) - yref(3,:), LineWidth=2);
        hold off;
        xlabel('x');
        ylabel('y');
        title('Pos Error');
        legend('x error', 'y error', 'z error');
        grid on;

        figure;
        plot3(pos_history(1,:), pos_history(2,:), pos_history(3,:))
        xlabel('x');
        ylabel('y');
        zlabel('z');
        
        % add plot of error over time betwen u
        % figure;
        % plot(u_given, 'LineWidth', 1.5);
        % xlabel('Iteration');
        % ylabel('input');
        % title('Input values');
        % grid on;


        stop(rc,  nhfc, maneuver);
    end
    % % set initial state
    % sim.set('x', x_sim(:,ii));
    % sim.set('u', u0);
    % % sim.set('xdot', zeros(nx,1)); % try to comment
    % sim.solve();
    % % get control input
    % %u_sim(:,ii) = sim.get('u');
    % % simulate system
    % x_sim(:,ii+1) = sim.get('xn');
    % u_sim(:,ii) = u0;%

    % for i = 1:13
    %     if abs(x_sim(i,ii+1)) < 1e-6
    %         x_sim(i,ii+1) = 0;
    %     end
    % end
    % error = norm(yref(3,ii) - x_sim(3,ii)) + norm(yref(2,ii) - x_sim(2,ii)) + norm(yref(2,ii) - x_sim(2,ii));
    % if ii > 1
    %     delta_u = u_sim(1,ii-1) - u_sim(1,ii);
    % end
    % toc
    % 
    % 
    % if ii < N
    %     ii = ii+1;
    % end


end