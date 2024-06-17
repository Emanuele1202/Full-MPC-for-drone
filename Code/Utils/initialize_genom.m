function [robot, rc, opti, pom, maneuver, nhfc] = initialize_genom()


%% Load robot parameters
% Read the configuration file
robot = genom_quadrotor();

 

%% Initialize Genom components
disp(['Connect to hostname: ' robot.hostname]);
g = genomix.client(robot.hostname);     
g.rpath([robot.openrobots_dir '/lib/genom/pocolibs/plugins']);
g.rpath([robot.devel_dir '/lib/genom/pocolibs/plugins']);


%% Load components
rc = g.load('rotorcraft');
opti = g.load('optitrack');
pom = g.load('pom');
maneuver = g.load('maneuver');
nhfc = g.load('nhfc');

end

