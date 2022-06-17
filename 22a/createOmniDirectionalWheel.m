function omnidirwheel_submb...
    = createOmniDirectionalWheel(wheel_r,wheel_thickness,roller_r,roller_l,roller_ang,roller_num)
% CREATEOMNIDIRECTIONALWHEEL is a fcn to build the omniDirectional wheels 
% for the mobile robot.

% wheel_r : Wheel Radius (1x1 simscape Value)
% wheel_thickness : Thickness of the wheel (1x1 simscape Value)
% roller_r : Radius of the rollers on the wheels (1x1 simscape Value)
% roller_l : Length of the rollers on the wheels (1x1 simscape Value)
% roller_ang : Orientation of the rollers on each of the 4 wheels (4x1
% simscape Value)
% roller_num : Number of rollers per wheel (1x1 simscape Value)

% Copyright 2021 The MathWorks, Inc.

% Import packages for using the MATLAB Interface to SM and Simscape Value
import simscape.multibody.* ;
import simscape.* ;

omnidirwheel_submb = Multibody;

wheel_solid  = makeWheel(wheel_r,wheel_thickness);

addComponent(omnidirwheel_submb,'Wheel' , wheel_solid);

% Add Rigid Transform 
 k = 1;
 if roller_num > 0
     for theta = 0:(2*pi)/roller_num:(2*pi-deg2rad(1))

         [roller_mb,slot_dim] = ...
             makeRoller(wheel_r,wheel_thickness,roller_r,roller_l,k,roller_ang);

         addComponent(omnidirwheel_submb,['roller',num2str(k)] , roller_mb);

         % Wheel To Roller Base RTs
         ang = simscape.Value(theta,'rad');
         axis = Axis.PosZ;
         rt_Rot = StandardAxisRotation(ang,axis);

         rot_wheelroller = RigidTransform(rt_Rot);
         addComponent(omnidirwheel_submb,['rot_wheelslot',num2str(k)] , rot_wheelroller);

         offset = wheel_r + slot_dim(2)/2;
         trans_axis = Axis.PosY;
         rt_Trans = StandardAxisTranslation(offset,trans_axis);

         trans_wheelroller = RigidTransform(rt_Trans);
         addComponent(omnidirwheel_submb,['trans_wheelslot',num2str(k)] , trans_wheelroller);

         % ......Connect Blocks.........
         connect(omnidirwheel_submb,['Wheel','/','R'],[['rot_wheelslot',num2str(k)],'/','B']);
         connect(omnidirwheel_submb,[['rot_wheelslot',num2str(k)],'/','F'],[['trans_wheelslot',num2str(k)],'/','B']);
         connect(omnidirwheel_submb,[['trans_wheelslot',num2str(k)],'/','F'],[['roller',num2str(k)],'/',roller_mb.FrameConnectors{1}]);

         k = k+1;
     end
 end
% Add and Connect
addConnector(omnidirwheel_submb,'WheelFrame','Wheel/R');

end

function wheel_solid  = makeWheel(wheel_r,wheel_thickness)
% MAKEWHEEL creates a cylindrical wheel solid

import simscape.multibody.* ;
import simscape.* ;
% Main Wheel
main_wheel = Cylinder(wheel_r,wheel_thickness);
main_wheel_viz = SimpleVisualProperties([0.2 0.2 0.2],1);
wheel_solid = Solid(main_wheel,main_wheel_viz);

end

function roller_slot_solid = makeRollerSlot(slot_dim)
% MAKEROLLERSLOT creates a brick solid representing a slot for the roller

import simscape.multibody.* ;
import simscape.* ;

roller_slot = Brick(slot_dim);
roller__slot_viz = SimpleVisualProperties([0.5 0.1 0],1);
roller_slot_solid = Solid(roller_slot,roller__slot_viz);

end

function [roller_mb,slot_dim] = makeRoller(wheel_r,wheel_thickness,roller_r,roller_l,indx,roller_ang)
% MAKEROLLER creates a roller multibody object 
import simscape.multibody.* ;
import simscape.* ;

k = indx;

roller_mb = Multibody;

%...............
slot_dim = [wheel_r/20, wheel_r/6 ,wheel_thickness];
roller_slot_solid = makeRollerSlot(slot_dim);
addComponent(roller_mb,['slot',num2str(k)] , roller_slot_solid);
%..............

roller_cyl = Cylinder(roller_r,roller_l);
roller_viz = SimpleVisualProperties([1 1 0],1);
roller_solid = Solid (roller_cyl,roller_viz);

addComponent(roller_mb,['roller',num2str(k)] , roller_solid);
%................
offset =  slot_dim(2)/2;
trans_axis = Axis.PosY;
rt_Trans = StandardAxisTranslation(offset,trans_axis);
trans_slotroller = RigidTransform(rt_Trans);

addComponent(roller_mb,['trans_slotroller',num2str(k)] , trans_slotroller);
%.............
ang = roller_ang;
axis = Axis.PosY;
rt_Rot = StandardAxisRotation(ang,axis);

rot_slotroller = RigidTransform(rt_Rot);

addComponent(roller_mb,['rot_slotroller',num2str(k)] , rot_slotroller);
%.............
connect(roller_mb,[['slot',num2str(k)],'/','R'],[['trans_slotroller',num2str(k)],'/','B']);
connect(roller_mb,[['trans_slotroller',num2str(k)],'/','F'],[['rot_slotroller',num2str(k)],'/','B']);
%.........
revJ = RevoluteJoint;
addComponent(roller_mb,['revJ',num2str(k)] , revJ);
%.........
connect(roller_mb,[['rot_slotroller',num2str(k)],'/','F'],[['revJ',num2str(k)],'/','B']);
connect(roller_mb,[['revJ',num2str(k)],'/','F'],[['roller',num2str(k)],'/','R']);
%.......
addConnector(roller_mb,'RollerSlotFrame',[['slot',num2str(k)],'/R']);

end



