classdef move_robot < handle
    %ROBOTFUNCTIONALITY 
    %class handles the robots motion and function.
    % The robot picks up bricks and places them at defined locations.
    
    properties
    end
    
    methods
    end
    
    methods (Static)
        function robot()
            % Clear workspace and close all figures
            clear all;
            close all;
            clf;

            % Populate MATLAB environment with safety equipment
            WorkSpaceEnv.Run();

            % Assigning initial brick locations to the matrix
            brickMatrix = zeros(1,3);
            brickMatrix(1,:) = [-2.8, 0.42, 0.7];

                        
            % Assigning final brick locations to the matrix
            finalBrickMatrix = zeros(1,3);
            finalBrickMatrix(1,:) = [0.4, 0, 0.72];
           
            
            % Define the number of bricks
            numBricks = 1;
            bricks = cell(numBricks, 1);
            
            % Create and place bricks at their initial locations
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('HalfSizedRedGreenBrick.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(brickMatrix(brickIndex,:))';
                set(bricks{brickIndex},'Vertices',transformedVertices(:,1:3));
            end
            
            % Place robot model on table
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);
            r.model;
            
            % Gripper definition (two fingers with two segments each)
            finger_length = 0.05;
            second_link_length = 0.03;
            gripper_finger_spacing = 0.08;  % Adjusted spacing

            % Create two fingers for the gripper (each with 2 links)
            finger1 = SerialLink([
                Revolute('d', 0, 'a', finger_length, 'alpha', pi/2, 'standard')  % First segment
                Revolute('d', 0, 'a', second_link_length, 'alpha', 0, 'standard')  % Second segment
                ]);
            
            finger2 = SerialLink([
                Revolute('d', 0, 'a', finger_length, 'alpha', pi/2, 'standard')  % First segment
                Revolute('d', 0, 'a', second_link_length, 'alpha', 0, 'standard')  % Second segment
                ]);

            % Get initial starting robot position
            initialPosition = r.model.getpos();

            % Loop for each brick
            for brickIndex = 1:numBricks
                % Get the initial and final brick location
                currentBrick = brickMatrix(brickIndex, :);
                finalBrick = finalBrickMatrix(brickIndex, :);

                % Define trajectory step count
                count = 100;

                % Get the robot's current arm location
                robotLocation = r.model.getpos();
                
                % Calculate joint angles for the robot to reach the current brick
                currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));  % Rotate the brick grabbing

                % Create joint space trajectory for the robot to move to the brick
                currentQPath = jtraj(robotLocation, currentBrickPath, count);

                % Animate the robot to the current brick
                for i = 1:size(currentQPath, 1)
                    % Animate the robot arm
                    r.model.animate(currentQPath(i, :));

                    % Get end effector transform
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                    % Attach the gripper fingers to the robot's end effector, rotating them to face forward
                    % Finger 1 on the left (-x direction), rotated to face forward
                    finger1_base_transform = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * troty(-pi / 2);  % Rotate 90 degrees about y-axis
                    finger1.base = finger1_base_transform;
                    finger1.plot([0, 0]);  % Open position
                    drawnow();
                    
                    % Finger 2 on the right (+x direction), rotated to face forward
                    finger2_base_transform = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * troty(-pi / 2);  % Rotate 90 degrees about y-axis
                    finger2.base = finger2_base_transform;
                    finger2.plot([0, 0]);  % Open position

                    drawnow();
                end

                % Simulate gripping the brick by closing the fingers
                fingerCloseAngle = 0.2;  % Adjusted to avoid brick collisions
                secondJointCloseAngle = 0.3;  % Adjusted to avoid collisions

                % Animate the gripper closing around the brick
                finger1.plot([fingerCloseAngle, secondJointCloseAngle]);  % Close finger 1
                finger2.plot([-fingerCloseAngle, -secondJointCloseAngle]);  % Close finger 2

                % Move the brick to the robot's end effector
                % Center the brick between the fingers
                brickOffset = transl(0, 0, -0.05);  % Adjust as needed for proper alignment

                % Now, move to the final brick location
                finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));

                % Create joint space trajectory for the robot to move to the final brick location
                finalQPath = jtraj(currentBrickPath, finalBrickPath, count);

                % Animate the robot to the final brick location
                for i = 1:size(finalQPath, 1)
                    % Animate the robot arm
                    r.model.animate(finalQPath(i, :));

                    % Get end effector transform
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                    % Update gripper finger positions (still attached to the end effector)
                    % Finger 1 on the left
                    finger1.base = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * troty(pi / 2);
                    finger1.plot([fingerCloseAngle, secondJointCloseAngle]);  % Closed finger 1

                    % Finger 2 on the right
                    finger2.base = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * troty(pi / 2);
                    finger2.plot([-fingerCloseAngle, -secondJointCloseAngle]);  % Closed finger 2

                    % Move the brick with the robot's end effector (center it between the fingers)
                    brickTransform = endEffectorTransform * brickOffset;
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)] * brickTransform';
                    set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));

                    drawnow();
                end

                % After moving, simulate releasing the brick by opening the fingers
                finger1.plot([0, 0]);  % Open finger 1
                finger2.plot([0, 0]);  % Open finger 2
            end
            
            % Return robot to its initial position
            count = 100;
            robotLocation = r.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
                r.model.animate(QPath(i, :));
                drawnow();
            end
        end
    end
end
