% BUILDMOBILEROBOT is script which makes a MobileRobot with 4 Omni Directional
% Mecanum Wheels and a manipulator.

% This script invokes the fcn makeMobileRobot() to do most of the work.
% The script's primary purpose is to set up the arguments to this function.

% Copyright 2021 The MathWorks, Inc.

%....................Mobile Robot Parameters..............................%

% Wheel Parameters
wheel_thickness = simscape.Value(0.07,'m');
wheel_r = simscape.Value(0.05,'m');
axle_r = simscape.Value(0.02,'m');
axle_l = simscape.Value(0.15,'m');

% Wheel Roller Parameters
num_of_rollers = 12; % Num of Rollers on an omniDirectionalWheel
roller_r = simscape.Value(0.01,'m');
roller_l = simscape.Value(0.08,'m');

% Specify roller angle : pi/4 for meccanum wheels and pi/2 for regular omni
% directional wheels.

roller_ang = {simscape.Value(-pi/4,'rad');...Wheel1
              simscape.Value(pi/4,'rad');... Wheel2
              simscape.Value(pi/4,'rad');... Wheel3
              simscape.Value(-pi/4,'rad')};% Wheel4

% Mobile Robot chasis parameters
chasis_dim = simscape.Value([0.75 0.3 0.1 ],'m');

%....................Robot Arm Parameters.................................%

% Arm Base
armBaseParams.r = simscape.Value(0.75,'m')/10 ;
armBaseParams.l = simscape.Value(0.5,'m')/10 ;

% Arm Links
l = {simscape.Value(2,'m')/10;simscape.Value(2,'m')/10;simscape.Value(1,'m')/10};
w = {simscape.Value(0.5,'m')/10;simscape.Value(0.5,'m')/10;simscape.Value(0.5,'m')/10};
t = {simscape.Value(0.3,'m')/10;simscape.Value(0.3,'m')/10;simscape.Value(0.3,'m')/10};
color = {[1.0 0.4 0.4],[1.0 0.4 0.4],[1.0 0.4 0.4]};

for i = 1:length(l)
    armLinkParams(i).l = l{i};
    armLinkParams(i).w = w{i};
    armLinkParams(i).t = t{i};
    armLinkParams(i).color = color{i};
end

% EndEffector 
endEffParams.r = simscape.Value(0.25,'m')/10;
endEffParams.l = simscape.Value(0.25,'m')/10 ;
endEffParams.color = [1.0 0.4 0.4];

% Gripper
gripperParams.dim = simscape.Value([0.8 0.2 0.05],'m')/10 ;
gripperParams.color = [0.2 0.2 0.2];

% Call makeMobileRobotFcn
[mobileRobot_mb,op] = makeMobileRobot(wheel_r,...
                                      wheel_thickness,...
                                      roller_r,...
                                      roller_l,...
                                      roller_ang,...
                                      num_of_rollers,...
                                      chasis_dim,...
                                      axle_r,...
                                      axle_l,...
                                      armBaseParams,...
                                      armLinkParams,...
                                      endEffParams,...
                                      gripperParams);

% Visualize
cmb = compile(mobileRobot_mb);
visualize(cmb,computeState(cmb,op),'Viz');
