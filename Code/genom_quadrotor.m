function [robot] = genom_quadrotor()
    name = 'quad';
    robot.name = name;
    robot.hostname = 'localhost';
    robot.openrobots_dir = '/home/gcorsini/AR_group/openrobots';
    robot.devel_dir = '/home/gcorsini/AR_group/devel';
    robot.log_path = './../logs/'; % directory for log files, relative to launch files
    robot.log_df = 1; % decimation factor (1: all samples, 2: half, 3: 1 out of 3, ...)

    robot.body.np = 4; % geometrical and inertial
    robot.body.m = 1.00 + 0.07 * 4; % [kg] robot mass = 1.00 (body mass) + 4*0.07 (rotor masses)
    robot.body.Jxx = 0.015; % [kgm^2] inertia (assumed diagonal)
    robot.body.Jyy = 0.015; 
    robot.body.Jzz = 0.007;
    robot.body.al = 0.23; % [m] arm length
    robot.body.cf = 6.5e-4; % [N/hz^2] thrust coeff
    robot.body.ct = 1.0e-5; % [Nm/Hz^2] drag coefftorque 
    robot.body.alpha = 0; % [deg] alpha radial tilting angle of rotors
    robot.body.beta = 0; % [deg] beta tangential tilting angle of rotors
    robot.body.wmin = 16;  % [Hz] min prop rotation speed
    robot.body.wmax = 140;  % [Hz] max prop rotation speed

    robot.opti.host = 'localhost'; % optitrack motion capture
    robot.opti.port = '1509';
    robot.opti.mcast = '';
    robot.opti.mcast_port = '0';

    robot.rc.port = '/tmp/pty-quad'; % rotorcraft
    robot.rc.baud = 0; % a.k.a. 'do-not-change' (see rotorcraft-genom3)
    robot.rc.rimu = 1000; % [Hz] imu rate
    robot.rc.rmag = 0; % [Hz] magnetometer rate
    robot.rc.rmot = 100; % [Hz] motor rate
    robot.rc.rbat = 1; % [Hz] battery rate
    robot.rc.gfc = {20, 20, 20}; % [Hz] gyroscope cut-off frequency
    robot.rc.afc = {5, 5, 5}; % [Hz] accelerometer cut-off frequency
    robot.rc.mfc = {20, 20, 20}; % [Hz] magnetometer cut-off frequency

    robot.nhfc.kp = [5, 5];
    robot.nhfc.kv = [6, 6];
    robot.nhfc.kpi = [0, 0]; % 5 5
    robot.nhfc.kq = [4, 4];
    robot.nhfc.kw = [1, 1];
    robot.nhfc.kqi = [0, 0]; % 5 5
end