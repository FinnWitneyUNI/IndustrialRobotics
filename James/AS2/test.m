classdef test < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            
            profile on;
            
            % Calls method run from workspaceenv to set up the environment
            % (Replace with your environment setup code if necessary)
            % WorkSpaceEnv.Run()  % Commented out for simplicity

            % Set up the UR3e robot model
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);  % UR3e robot
            r.model;

            % Gripper setup (for UR3e)
            gripper_finger_spacing = 0.1;  % Adjustment to prevent overlap
            finger1 = LinearFinger();  % Left finger of the gripper
            finger2 = LinearFinger();  % Right finger of the gripper

            % **Attach gripper fingers to UR3e end-effector immediately**
            % Get the UR3e end-effector transform
            ur3eEndEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Position the left gripper finger (-x direction relative to the end-effector)
            baseTr1 = ur3eEndEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
            finger1.model.base = baseTr1;
            finger1.model.animate([0, 0]);

            % Position the right gripper finger (+x direction relative to the end-effector)
            baseTr2 = ur3eEndEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(pi/2);
            finger2.model.base = baseTr2;
            finger2.model.animate([0, 0]);

            % Move the UR3e robot to a new position and update the gripper
            count = 100;  % Number of steps for the movement
            q_initial = r.model.getpos();  % Initial joint configuration
            q_final = [pi/4, -pi/4, pi/4, -pi/4, pi/4, 0];  % Example final joint configuration

            % Generate a joint trajectory
            qTrajectory = jtraj(q_initial, q_final, count);

            % Animate the UR3e robot and update the gripper fingers
            for i = 1:size(qTrajectory, 1)
                r.model.animate(qTrajectory(i, :));  % Animate UR3e robot

                % Get the updated end-effector transformation
                ur3eEndEffectorTransform = r.model.fkine(r.model.getpos()).T;

                % Update gripper finger positions relative to the end-effector
                baseTr1 = ur3eEndEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
                finger1.model.base = baseTr1;
                finger1.model.animate([0, 0]);

                baseTr2 = ur3eEndEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(pi/2);
                finger2.model.base = baseTr2;
                finger2.model.animate([0, 0]);

                % Redraw the scene
                drawnow();
            end
            
            % Debugging: Display final position of UR3e's end-effector
            disp('Final End-Effector Position:');
            disp(ur3eEndEffectorTransform(1:3, 4));  % Display the position (x, y, z)
        end
    end
end
