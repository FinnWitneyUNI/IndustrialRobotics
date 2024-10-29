rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');

jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

%% 

openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

q0 = [58.56,-110.99,55.78,-92.89,-89.68,147.65];
q0 = deg2rad(q0);
q1 = [54.34,-90.36,101.31,-99.21,-92.45,143.29];
q1 = deg2rad(q1);
q2 = [55.35,-91.59,91.40, -88.03,-92.40,144.29];
q2 = deg2rad(q2);
q3 =[82.31,-89.60,89.06,-86.71,-91.27,171.25];
q3 = deg2rad(q3);
q4 = [82.32,-88.37,101.04,-99.93,-91.29,171.27];
q4 = deg2rad(q4);


jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.TimeFromStart = rosduration(0); 
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

closeService.call();
pause(5);
openService.call();
pause(5);

startJointSend.Positions = currentJointState_123456;
nextJointState_123456 = q1;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);   %% getting sent to pi 
pause(5);

startJointSend.Positions = q1;
nextJointState_123456 = q2;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
closeService.call();
sendGoal(client,goal);
pause(5)%% getting sent to pi 
% closeService.call();

startJointSend.Positions = q2;
nextJointState_123456 = q3;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
openService.call();
sendGoal(client,goal);
pause(5);
 
startJointSend.Positions = q3;
nextJointState_123456 = q4;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);
goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
openService.call();
sendGoal(client,goal);
pause(5);


% openService.call();
% closeService.call();


% %%
% r = UR3()
% 
% 
% q0 = [58.56,-110.99,55.78,-92.89,-89.68,147.65];
% q0 = deg2rad(q0);
% q1 = [54.34,-90.36,101.31,-99.21,-92.45,143.29];
% q1 = deg2rad(q1);
% q2 = [55.35,-91.59,91.40, -88.03,-92.40,144.29];
% q2 = deg2rad(q2);
% q3 =[82.31,-89.60,89.06,-86.71,-91.27,171.25];
% q3 = deg2rad(q3);
% q4 = [82.32,-88.37,101.04,-99.93,-91.29,171.27];
% q4 = deg2rad(q4);
% q = zeros(1,6);
% qmatrix0 = jtraj(q,q0,50);
%  qmatrix = jtraj(q0,q1,50);
%  qmatrix1 = jtraj(q1,q2,50);
%  qmatrix2 = jtraj(q2,q3,50);
%  r.model.plot(qmatrix0);
%  r.model.plot(qmatrix);
%     r.model.plot(qmatrix1);
%     r.model.plot(qmatrix2);