function chassis_mb = createMobileRobotChassis(chassis_dim,axle_r,axle_l)
% CREATEMOBILEROBOTCHASSIS is a fcn to build the chassis of the mobile
% robot.

% chassis_dim : Dimension of brick chassis 3x1 vector (simscape.Value)
% axle_r : Radius of the cylindrical axle (simscape.Value) 
% axle_l : Length of cylindrical axle (simscape.Value)

% Copyright 2021 The MathWorks, Inc.

% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

chassis_mb = Multibody;

chassis_rb = RigidBody;

chassis = Brick(chassis_dim);
chassis_viz = SimpleVisualProperties([0.2 0.2 1],1);
chassis_solid = Solid(chassis,chassis_viz);

addComponent(chassis_rb,'chassis' , 'reference',chassis_solid);
addConnector(chassis_rb,'reference');

wheel_offset = ...
    {[chassis_dim(1)*0.4, chassis_dim(2)/2, -chassis_dim(3)*0.3];...
    [chassis_dim(1)*0.4, -chassis_dim(2)/2, -chassis_dim(3)*0.3];...
    [-chassis_dim(1)*0.4, chassis_dim(2)/2, -chassis_dim(3)*0.3];...
    [-chassis_dim(1)*0.4, -chassis_dim(2)/2, -chassis_dim(3)*0.3]};

for k = 1:4
    % Rigid Transforms
    rt_Trans = CartesianTranslation(wheel_offset{k});

    ang = simscape.Value(pi/2,'rad');
    axis = Axis.PosX;
    rt_Rot = StandardAxisRotation(ang,axis);
    chassis_wheel_rt1 = RigidTransform(rt_Rot,rt_Trans);

    addFrame(chassis_rb,['RT',num2str(k)] , 'reference' , chassis_wheel_rt1);

    addConnector(chassis_rb,['RT',num2str(k)]);

end

chassis2arm_rt = RigidTransform(...
    CartesianTranslation(...
    [chassis_dim(1)/3 0 chassis_dim(3)/2]));

addFrame(chassis_rb,'chassis2arm_rt' , 'reference' , chassis2arm_rt);
addConnector(chassis_rb,'chassis2arm_rt');

addComponent(chassis_mb,'chassis' , chassis_rb);
addConnector(chassis_mb,'ref','chassis/reference');
addConnector(chassis_mb,'ArmB','chassis/chassis2arm_rt');

wheelF =  {[simscape.Value([0 0],'m') , -axle_l/2];...
    [simscape.Value([0 0],'m') , axle_l/2];...
    [simscape.Value([0 0],'m') , -axle_l/2];...
    [simscape.Value([0 0],'m') , axle_l/2]};

for k = 1:4

    axle_cyl = Cylinder(axle_r,axle_l);
    axle_viz = SimpleVisualProperties([0.5 1 0.5],1);
    axle_solid = Solid (axle_cyl,axle_viz);
    addComponent(chassis_mb,['axle',num2str(k)] , axle_solid);

    wheelTrans = CartesianTranslation(wheelF{k});
    rt_WheelF = RigidTransform(wheelTrans);
    addComponent(chassis_mb,['W',num2str(k)] , rt_WheelF);

    connect(chassis_mb,['chassis','/',['RT',num2str(k)]],[ ['axle',num2str(k)],'/','R']);
    connect(chassis_mb,[['axle',num2str(k)],'/','R'],[['W',num2str(k)],'/','B']);

    addConnector(chassis_mb,['WheelF',num2str(k)],[['W',num2str(k)],'/F']);
    
end
end
