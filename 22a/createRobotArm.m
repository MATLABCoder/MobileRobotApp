function arm_mb = createRobotArm(ArmBaseParams,ArmLinkParams,EndEffParams,GripperParams)
% CREATEROBOTARM makes the manipulator for the mobile robot

% ArmBaseParams : Struct with different arm base parameters like
% radius,length,color 
% ArmLinkParams : Struct with different arm link parameters like
% dimensions, color
% EndEffParams : Struct with different end effector parameters
% Gripper Params : Struct with different gripper parameters

% Copyright 2021 The MathWorks, Inc.

% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

arm_mb = Multibody;

% ................Create Arm Base ..........................%

link1_rb = createArmBase(ArmBaseParams.r,ArmBaseParams.l);
% Add Baselink
addComponent( arm_mb,'BaseLink' , link1_rb);
addConnector(arm_mb,'ArmBase','BaseLink/reference');

%.................Create Arm Links .........................................%
num_of_arms = length(ArmLinkParams);

for i =1:num_of_arms
    
    arm_links_rb = createArmLinks(ArmLinkParams(i).l,...
                                  ArmLinkParams(i).w,...
                                  ArmLinkParams(i).t,...
                                  ArmLinkParams(i).color);
    addComponent( arm_mb,['arm_link_',num2str(i)] , arm_links_rb);
    
end

% ...............Created EndEffector Base..................................%
t = ArmLinkParams(end).t;
l = ArmLinkParams(end).l;

fixture_solid = Solid(...
    Brick([t t t]),...   
    SimpleVisualProperties([0.2 0.2 0.2],1));

fixture_rt = RigidTransform(...
    StandardAxisTranslation(...
    t/2,Axis('PosZ')));

addComponent(arm_mb,'fixture' , fixture_solid);
addComponent(arm_mb,'fixture_rt' , fixture_rt);

fixture2rev_rt = RigidTransform(...
    StandardAxisRotation(...
    simscape.Value(pi/2,'rad'),Axis('PosY')),...
    StandardAxisTranslation(...
    t/2,Axis('PosX')));

addComponent(arm_mb,'fixture2rev_rt' , fixture2rev_rt);

connect(arm_mb,['arm_link_3','/','PosEnd'],['fixture_rt','/','B']);
connect(arm_mb,['fixture_rt','/','F'],['fixture','/','R']);
connect(arm_mb,['fixture','/','R'],['fixture2rev_rt','/','B']);

%........................Create EndEffector..................................%

arm_endEff_rb = createEndEffector(EndEffParams.r,EndEffParams.l,EndEffParams.color);
addComponent(arm_mb,'EndEffector' , arm_endEff_rb);

%........................Create Gripper Base.................................%
gripper_base_dim = [EndEffParams.r*4 EndEffParams.r/2 EndEffParams.r/4];
arm_gripper_base_rb = createGripperBase(gripper_base_dim);
addComponent(arm_mb,'gripperBase' , arm_gripper_base_rb);

%.......................Create Gripper Fingers..............................%

arm_gripper_rb = createGripperFingers(GripperParams.dim,GripperParams.color);
addComponent(arm_mb,'gripperFing1' , arm_gripper_rb);
addComponent(arm_mb,'gripperFing2' , arm_gripper_rb);

connect(arm_mb,['EndEffector','/','basePort2'],['gripperBase','/','ee2gr_rt']);
% add Prismatic Joint between the grippers
prisJ = PrismaticJoint;

addComponent(arm_mb,'Pris1' , prisJ);
addComponent(arm_mb,'Pris2' , prisJ);

connect(arm_mb,['gripperBase','/','gr2gr1_rt'],['Pris1','/','B']);
connect(arm_mb,['Pris1','/','F'],['gripperFing1','/','gripper_B']);
connect(arm_mb,['gripperBase','/','gr2gr2_rt'],['Pris2','/','B']);
connect(arm_mb,['Pris2','/','F'],['gripperFing2','/','gripper_B']);

%......................Connect Joints..............................%

for i = 2:5
    revJ = RevoluteJoint;
    addComponent(arm_mb,['Rev_',num2str(i)] , revJ);
end

connectVia(arm_mb,'Rev_2','BaseLink/link1b_2_rt','arm_link_1/NegEnd');

addComponent(arm_mb,...
    'NegEnd_RT',...
    RigidTransform(...
    CartesianTranslation(...
    [0, 0 , -ArmLinkParams(1).t])));

connectVia(arm_mb,'Rev_3','arm_link_1/PosEnd','NegEnd_RT/B');
connect(arm_mb,['NegEnd_RT','/','F'],['arm_link_2','/','NegEnd']);

addComponent(arm_mb,...
    'NegEnd_RT1',...
    RigidTransform(...
    CartesianTranslation(...
    [0, 0 , -ArmLinkParams(1).t])));

connectVia(arm_mb,'Rev_4','arm_link_2/PosEnd','NegEnd_RT1/B');
connect(arm_mb,'NegEnd_RT1/F','arm_link_3/NegEnd')

connectVia(arm_mb,'Rev_5','fixture2rev_rt/F','EndEffector/endEff2Fix_rt');

end

%.............................Local Functions............................................%

function  link1_rb = createArmBase(link1_r,link1_l)
% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

link1_rb = RigidBody;

link1_geom = Cylinder(link1_r,link1_l);
link1_viz = SimpleVisualProperties([1.0 0.4 0.4],1);
link1_solid = Solid(link1_geom,link1_viz);

addComponent(link1_rb,'BaseLink' , 'reference',link1_solid);

link1b_dim = simscape.Value([0.1 0.5 0.5],'m')/10;
link1b_geom = Brick(link1b_dim);
link1b_viz = SimpleVisualProperties([0.2 0.2 0.2],1);
link1b_solid = Solid(link1b_geom,link1b_viz);

offset = link1_l/2 + link1b_dim(3)/2;
axis = Axis('PosZ');
trans = StandardAxisTranslation(offset,axis);
link1_1b_rt = RigidTransform(trans);

addFrame(link1_rb,'link1_1b_rt' , 'reference' , link1_1b_rt);

addComponent(link1_rb,'BaseLink_B' , 'link1_1b_rt',link1b_solid);

ang = simscape.Value(pi/2,'rad');
axis = Axis('PosY');
rot = StandardAxisRotation(ang,axis);
offset = simscape.Value(0.35,'m')/10;
axis = Axis('PosX');
trans = StandardAxisTranslation(offset,axis);
link1b_2_rt = RigidTransform(rot,trans);

addFrame(link1_rb,'link1b_2_rt' , 'link1_1b_rt' , link1b_2_rt);

addConnector(link1_rb,'link1b_2_rt');
addConnector(link1_rb,'reference');

end

function arm_links_rb = createArmLinks(l,w,t,color)
% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

arm_links_rb = RigidBody;

xs = roundedRect(l, w);

ge = GeneralExtrusion(xs,t);
ge_viz = SimpleVisualProperties(color,1);
link_Solid = Solid(ge,ge_viz);

addComponent(arm_links_rb,'arm_link' , 'reference',link_Solid);

addFrame(arm_links_rb,'PosEnd','reference',...
    RigidTransform(...
    CartesianTranslation(...
    [l/2, 0 , t/2])));

addFrame(arm_links_rb,'NegEnd','reference',...
    RigidTransform(...
    CartesianTranslation(...
    [-l/2, 0 , t/2])));

addConnector(arm_links_rb,'PosEnd');
addConnector(arm_links_rb,'NegEnd');

    function xs = roundedRect(length, width)
        % Return the cross section of a rectangle with rounded ends
        angles = (-90:10:+90)' * pi/180;
        semi = width/2 * [cos(angles) sin(angles)] + repmat([length/2 0], size(angles));
        xs = [semi; -semi];
    end

end

function arm_endEff_rb = createEndEffector(endEff_r,endEff_l,endEff_color)
% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

arm_endEff_rb = RigidBody;

arm_endEff_solid = Solid(...
    Cylinder(endEff_r,endEff_l),...    
    SimpleVisualProperties([0.2 1 0.2],1));

endEff2Fix_rt = RigidTransform(...
    StandardAxisTranslation(...
    -endEff_l/2,Axis('PosZ')));

endEff2gripper_rt = RigidTransform(...
    StandardAxisTranslation(...
    endEff_l/2,Axis('PosZ')));

arm_gripper_solid = Solid(...
    Cylinder(endEff_r*2,endEff_l),...    
    SimpleVisualProperties(endEff_color,1));

basePort1 = RigidTransform(...
    StandardAxisTranslation(...
    +endEff_l/2,Axis('PosZ')));

basePort2 = RigidTransform(...
    StandardAxisTranslation(...
    +endEff_l/2,Axis('PosZ')));

addComponent(arm_endEff_rb,'endEff' , 'reference',arm_endEff_solid);
addFrame(arm_endEff_rb,'endEff2gripper_rt' , 'reference' , endEff2gripper_rt);
addFrame(arm_endEff_rb,'endEff2Fix_rt' , 'reference' , endEff2Fix_rt);
addFrame(arm_endEff_rb,'basePort1' , 'endEff2gripper_rt' , basePort1);
addComponent(arm_endEff_rb,'gripperBase' , 'basePort1',arm_gripper_solid);

addFrame(arm_endEff_rb,'basePort2' , 'basePort1' , basePort2);

addConnector(arm_endEff_rb,'basePort2');
addConnector(arm_endEff_rb,'endEff2Fix_rt');

end

function arm_gripper_base_rb = createGripperBase(gripper_base_dim)
% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

arm_gripper_base_rb = RigidBody;

gripper_base_solid = Solid(...
    Brick(gripper_base_dim),...   
    SimpleVisualProperties([0.2 0.2 0.2],1));

ee2gr_rt = RigidTransform(...
    StandardAxisTranslation(...
    gripper_base_dim(3)/2,Axis('NegZ')));

gr2gr1_rt = RigidTransform(...
    StandardAxisRotation(...
    simscape.Value(pi/2,'rad'),Axis('PosY')),...
    CartesianTranslation(...
    [gripper_base_dim(1)/2 0 gripper_base_dim(3)/2]));

gr2gr2_rt = RigidTransform(...
    StandardAxisRotation(...
    simscape.Value(pi/2,'rad'),Axis('PosY')),...
    CartesianTranslation(...
    [-gripper_base_dim(1)/2 0 gripper_base_dim(3)/2]));

addComponent(arm_gripper_base_rb,'gripper_base_solid' , 'reference',gripper_base_solid);
addFrame(arm_gripper_base_rb,'ee2gr_rt' , 'reference' , ee2gr_rt);
addFrame(arm_gripper_base_rb,'gr2gr1_rt' , 'reference' , gr2gr1_rt);
addFrame(arm_gripper_base_rb,'gr2gr2_rt' , 'reference' , gr2gr2_rt);

addConnector(arm_gripper_base_rb,'ee2gr_rt');
addConnector(arm_gripper_base_rb,'gr2gr1_rt');
addConnector(arm_gripper_base_rb,'gr2gr2_rt');

end

function  arm_gripper_rb = createGripperFingers(gripper_dim,gripper_color)
% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

arm_gripper_rb = RigidBody;

gripper_solid = Solid(...
    Brick(gripper_dim),...    
    SimpleVisualProperties(gripper_color,1));

gripper_rt_b = RigidTransform(...
    StandardAxisTranslation(...
    gripper_dim(1)/2,Axis('PosX')));

addComponent(arm_gripper_rb,'gripper' , 'reference',gripper_solid);
addFrame(arm_gripper_rb,'gripper_B' , 'reference' , gripper_rt_b);
addConnector(arm_gripper_rb, 'gripper_B');

end
