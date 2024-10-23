classdef here < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            hold on;
            view(3);
            axis([-2.5, 2.5, -2.5, 2.5, 0, 2.75]);
            % Set up the robot model (IIWA7)
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            i = IIWA7(defaultBaseTr);
            i.model.teach;

            % Check the number of DOF
            numDOF = length(i.model.links);
            disp(['Number of DOF: ', num2str(numDOF)]);

            % Check the joint limits
            qlim = i.model.qlim;  % Get joint limits for the IIWA7 robot
            disp('Joint limits for the robot:');
            disp(qlim);

            % Define the target end-effector position: (0.4, 0, 0.72)
            targetPosition = [0.4, 0, 0.72];

            % Create the homogeneous transformation matrix for the target position
            % Assuming the end-effector is oriented along the Z-axis
            targetTr = transl(targetPosition) * troty(pi);  % Rotation around Y-axis by 180 degrees

            % Get the current joint configuration
            initialQ = i.model.getpos();

            % Compute the inverse kinematics to get the joint angles for the target position
            % Use 'ikcon' to compute the joint angles for the target pose
            targetQ = i.model.ikcon(targetTr, initialQ);  % Inverse kinematics calculation

            % Check if any of the computed joint angles exceed the robot's joint limits
            for j = 1:numDOF
                if targetQ(j) < qlim(j, 1)
                    disp(['Joint ', num2str(j), ' is below the lower limit. Clipping.']);
                    targetQ(j) = qlim(j, 1);  % Clip to lower limit
                elseif targetQ(j) > qlim(j, 2)
                    disp(['Joint ', num2str(j), ' is above the upper limit. Clipping.']);
                    targetQ(j) = qlim(j, 2);  % Clip to upper limit
                end
            end

            % Generate a smooth trajectory from the initial configuration to the target configuration
            steps = 100;  % Number of steps for the animation
            qTrajectory = jtraj(initialQ, targetQ, steps);

            % Animate the robot moving to the target position
            for j = 1:steps
                i.model.animate(qTrajectory(j, :));
                pause(0.05);  % Small pause to make the animation smooth
            end
            
            disp('IIWA7 Robot end-effector moved to the target position.');
        end
    end
end
