% RunRMRC.m
clf; 
clear; 
close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

% Set up environment
WorkSpaceEnv.Run();

% Create robot
defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
robot = LinearUR3e(defaultBaseTr);

% Create controller
controller = RMRCController(robot.model, 0.05);

% Define start and end transforms
startTr = double(robot.model.fkine(robot.model.getpos()));
endTr = double(transl(0.4, -0.5, 0.8) * troty(pi/2));

% Make sure transforms are 4x4 homogeneous matrices
if ~all(size(startTr) == [4 4]) || ~all(size(endTr) == [4 4])
    error('Transforms must be 4x4 homogeneous matrices');
end

% Execute trajectory
[qMatrix, cartError] = controller.computeTrajectory(startTr, endTr, 10);

% Animate the movement
controller.animate(0.05);

% Plot the errors
controller.plotError();