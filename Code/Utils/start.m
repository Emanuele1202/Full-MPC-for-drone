function [] = start(hover_height, rc, nhfc, maneuver)
    if nargin < 1
        hover_height = 1.0; % Imposta l'altezza predefinita se non specificata
    end
    
    % input('Press ENTER to Start propellers');
    rc.start(); % spin propellers at lowest velocity (16 Hz)
    
    % README
    % This activity is required to start the control of the rotor speeds, 
    % as the measurements are based on sensing the back-EMF of the motors.
    % NB: In the Gazebo simulator, you will see the propellers spinning very slowly since the visualized velocities are downscaled for performance.

    % input('Press ENTER to Start controllers');
    maneuver.set_current_state();
    rc.servo('-a');
    
    fprintf('Robot is ready.\n');
    % input('Press ENTER to Take off');
    nhfc.servo('-a');
    % 
    % % ------ try to see rc send velocity duration ------%
    vel = {69.4949 , 69.4949, 69.4949, 69.4949, 0, 0, 0, 0};
    % vel_given = zeros(4, 1000);
    % rc.set_velocity(vel);
    % % a = nhfc.rotor_input();
    % i = 1;
    % pause(0.5)
    % 
    % while true
    % 
    %     rc.set_velocity(vel);
    %     % vel_given(:,i) = cell2mat(a.rotor_input.desired'); 
    %     % pause(0.05)
    % end
    % ------------------------------------------------- %
    
    maneuver.take_off(hover_height, 5);
    fprintf('Start done.\n');
    % maneuver.waypoint(2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 5);
    maneuver.desired()
end