function [mobileRobot_mb,op] = makeMobileRobot(...
                                             wheel_r,...
                                             wheel_thickness,...
                                             roller_r,...
                                             roller_l,...
                                             roller_ang,...
                                             num_of_rollers,...
                                             chassis_dim,...
                                             axle_r,...
                                             axle_l,...
                                             armBaseParams,...
                                             armLinkParams,...
                                             endEffParams,...
                                             gripperParams)

% MAKEMOBILEROBOT is a fcn which makes a MobileRobot with 4 Omni Directional
% Mecanum Wheels and a maniputlator using the MATLAB Interface to Simscape Multibody
% It takes in different struct parameters needed to make the robot.
%
% This function invokes : 

% createMobileRobotChassis() 
% createOmniDirectionlWheel()
% createManipulator()
% functions to create a robot Chassis, 4 Mecanum wheels and 
% manipulator.

% Copyright 2021 The MathWorks, Inc.

% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;
%.....................Code to Build the Mobile Robot....................%

% Initialize the Mobile Robot multibody with World Frame and Solver Config
% blocks.
mobileRobot_mb = Multibody;
addComponent(mobileRobot_mb,'World',WorldFrame())
% Create the Mobile Robot Chassis
Chassis_mb = createMobileRobotChassis(chassis_dim,axle_r,axle_l);
arm_mb = createRobotArm(armBaseParams,armLinkParams,endEffParams,gripperParams);

addComponent(mobileRobot_mb,'Chassis_Subsys' , Chassis_mb);
addComponent(mobileRobot_mb,'Arm_Subsys' , arm_mb);

RevJ_ArmB = RevoluteJoint;
addComponent(mobileRobot_mb,'RevJ_ArmB' , RevJ_ArmB);
connectVia(mobileRobot_mb,'RevJ_ArmB','Chassis_Subsys/ArmB','Arm_Subsys/ArmBase')

% Create and Add 4 OmniDirectionalWheels and connect it to the Chassis
for i = 1 : 4
    
omnidirwheel_submb...
    = createOmniDirectionalWheel(wheel_r,wheel_thickness,...
                                 roller_r,roller_l,roller_ang{i},num_of_rollers);
omniDirWheelSubsysName = ['omniWheel_',num2str(i)];
addComponent(mobileRobot_mb,omniDirWheelSubsysName , omnidirwheel_submb);
revJ = RevoluteJoint;
addComponent(mobileRobot_mb,['RevJ_',num2str(i)] , revJ);
connectVia(mobileRobot_mb,['RevJ_',num2str(i)],...
    ['Chassis_Subsys/',['WheelF',num2str(i)]],[omniDirWheelSubsysName,'/WheelFrame']);

end

% Add and connect the Mobile Robot to the World using a Planar Joint
Planar_Joint = PlanarJoint;
addComponent(mobileRobot_mb,'PlanarDofJ' , Planar_Joint);

connect(mobileRobot_mb,['World','/','W'],['PlanarDofJ','/','B']);
connect(mobileRobot_mb,['PlanarDofJ','/','F'],['Chassis_Subsys','/','ref']);

% Set Operating Point
op = simscape.op.OperatingPoint;
op("PlanarDofJ/Rz/w") = simscape.op.Target(10,'deg/s','High');
op("RevJ_1/Rz/w") = simscape.op.Target(5,'deg/s','High');
op("RevJ_4/Rz/w") = simscape.op.Target(15,'deg/s','High');
op("Arm_Subsys/Rev_1/Rz/q") = simscape.op.Target(60,'deg','High');
op("Arm_Subsys/Rev_2/Rz/q") = simscape.op.Target(120,'deg','High');
op("Arm_Subsys/Rev_3/Rz/q") = simscape.op.Target(60,'deg','High');
op("Arm_Subsys/Rev_4/Rz/q") = simscape.op.Target(-60,'deg','High');
op("Arm_Subsys/Rev_5/Rz/q") = simscape.op.Target(-60,'deg','High');

end
