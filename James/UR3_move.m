% Initialize the ROS connection to the Raspberry Pi
rosinit('192.168.27.1'); % Replace with the correct IP address of your Raspberry Pi
%%
% Subscribe to the joint states topic
jointStateSubscriber = rossubscriber('/ur/joint_states', 'sensor_msgs/JointState');
pause(2); % Pause to allow time for a message to be received

% Get the current joint positions
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];

% Define the joint names
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

%% 

%% 
% Create a client for the joint trajectory controller
[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now', 'system');
goal.GoalTimeTolerance = rosduration(0.05);
durationSeconds = 5; % This is how many seconds the movement will take
% Define the joint configurations in radians
q0 = [58.56,-110.99,55.78,-92.89,-89.68,147.65];
q0 = deg2rad(q0);

q1 = [54.34,-90.36,101.31,-99.21,-92.45,143.29];
q1 = deg2rad(q1);

q2 = [55.35,-91.59,91.40,-88.03,-92.40,144.29];
q2 = deg2rad(q2);

q3 =[82.31,-89.60,89.06,-86.71,-91.27,171.25];
q3 = deg2rad(q3);
%% 

openService.call();
pause(2);
closeService.call();
%% 

% Define the starting joint positions
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);

% Define the first target joint positions (q0)
firstJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
firstJointSend.Positions = q0;
firstJointSend.TimeFromStart = rosduration(durationSeconds); 

% Define the second target joint positions (q1)
secondJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
secondJointSend.Positions = q1;
secondJointSend.TimeFromStart = rosduration(durationSeconds*2);
% Define the third target joint positions (q2)
thirdJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
thirdJointSend.Positions = q2;
thirdJointSend.TimeFromStart = rosduration(durationSeconds*3);
% Define the third target joint positions (q3)
fourthJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
fourthJointSend.Positions = q3;
fourthJointSend.TimeFromStart = rosduration(durationSeconds*4);


% Set the trajectory points in the goal
goal.Trajectory.Points = [startJointSend; firstJointSend; secondJointSend; thirdJointSend; fourthJointSend];

% Send the trajectory command to the robot
bufferSeconds = 1; % Buffer time to account for network delays
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client, goal);

% waitForResult(client);
% 
% rosshutdown;
%%
openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

openService.call();
%closeService.call();