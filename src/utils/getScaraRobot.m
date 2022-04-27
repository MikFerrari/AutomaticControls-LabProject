function robot=getRobot()
link1_length=0.3;
link2_length=0.3;
robot = rigidBodyTree('MaxNumBodies',3,'DataFormat','column');


body1 = rigidBody('body1');
body1.Mass=2;
body1.CenterOfMass=[0,0.5*link1_length,0];
body1.Inertia(3)=2;

jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0,0,0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

addBody(robot,body1,'base')


body2 = rigidBody('body2');
body2.Mass=1;
body2.CenterOfMass=[0,0.5*link2_length,0];

jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform2 = trvec2tform([0,link1_length,0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1


bodyEndEffector = rigidBody('endeffector');
tform_ee = trvec2tform([0,link2_length,0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform_ee);
bodyEndEffector.Mass=1;
bodyEndEffector.Inertia(3)=1;

addBody(robot,bodyEndEffector,'body2'); % Add body2 to body1

figure
robot.show