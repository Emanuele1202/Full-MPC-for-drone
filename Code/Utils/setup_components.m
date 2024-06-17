
function setup_components(robot, rc, opti, pom, maneuver, nhfc)
    % ------------------------ Optitrack ------------------------
    
    msg_opti = opti.connect(robot.opti.host, robot.opti.port);
    msg_opti.wait(); % use send=true, or ack=true and then method 'wait' to wait for a status of an activity
    disp(['Connect to optitrack: ', num2str(msg_opti.status)]);

    % ------------------------ Rotorcraft ------------------------
    msg_rc = rc.connect(robot.rc.port, robot.rc.baud);
    % msg_rc.wait();
    disp(['Connect to flight controller: ', num2str(msg_rc.status)]);
    rc.set_sensor_rate(robot.rc.rimu, robot.rc.rmag, robot.rc.rmot, robot.rc.rbat);
    % Equivalent to...
    % rc.set_sensor_rate(
    %     rate={'imu', robot.rc.rimu, 'mag', robot.rc.rmag,
    %           'motor', robot.rc.rmot, 'battery', robot.rc.rbat});

    rc.set_imu_filter(robot.rc.gfc, robot.rc.afc, robot.rc.mfc); %-------problem
    rc.connect_port('rotor_input', 'nhfc/rotor_input');

    % ------------------------ Pom ------------------------
    pom.set_prediction_model('::pom::constant_acceleration');
    pom.set_process_noise(100, 50);
    % Equivalent to...
    % pom.set_process_noise('max_jerk', 100, 'max_dw', 50);
    pom.set_history_length(0.25);
    pom.connect_port('measure/opti', ['optitrack/bodies/', robot.name]);
    pom.add_measurement('opti',  0, 0, 0, 0, 0, 0);

    % ------------------------ Maneuver ------------------------
    maneuver.connect_port('state', 'pom/frame/robot');
    maneuver.set_bounds( -10, 10, -10, 10, 0, 10, 0, 2 * pi);

    % ====== nhfc ======
    nhfc.set_gtmrp_geom(robot.body.np, 0, 0, 0, robot.body.al, robot.body.m, robot.body.alpha, robot.body.beta, -1, robot.body.cf, robot.body.ct);
    nhfc.set_emerg(0.5, 10.5, 9.5, 10.5, 19.5);
    nhfc.set_saturation( 1,  1, 0);
    nhfc.set_servo_gain( robot.nhfc.kp(1), robot.nhfc.kp(2), ...
     robot.nhfc.kq(1),  robot.nhfc.kq(2), ...
     robot.nhfc.kv(1),  robot.nhfc.kv(2), ...
     robot.nhfc.kw(1),  robot.nhfc.kw(2), ...
     robot.nhfc.kqi(1), robot.nhfc.kqi(2));
    nhfc.set_control_mode('::nhfc::tilt_prioritized');
    nhfc.connect_port('state', 'pom/frame/robot');
    nhfc.connect_port('reference', 'maneuver/desired');
    disp('Setup done.');
end