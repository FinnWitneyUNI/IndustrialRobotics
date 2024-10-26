classdef main < handle

    properties
    end

    methods
    end

    methods (Static)
        function runTask(r, brickMatrix, finalBrickMatrix, gripper_finger_spacing)
            % Slightly increase gripper finger spacing to avoid collisions with objects
            gripper_finger_spacing = gripper_finger_spacing + 0.02;  % Adjustment to prevent overlap

            % Create two instances of the LinearFinger class (representing the robot's gripper fingers)
            finger1 = LinearFinger();  % Left finger of the gripper
            finger2 = LinearFinger();  % Right finger of the gripper

            % Define the total number of bricks to process, based on brickMatrix
            numBricks = size(brickMatrix, 1);  % Number of bricks is the number of rows in brickMatrix
            bricks = cell(numBricks, 1);  % Initialize a cell array to hold brick objects

            % Place each brick at its initial location based on the brickMatrix positions
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('HalfSizedRedGreenBrick.ply');  % Load the brick object
                vertices = get(bricks{brickIndex}, 'Vertices');  % Get the vertices of the brick object
                % Transform the brick's vertices to place it at the position specified by brickMatrix
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex,:))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));  % Apply the transformation
            end

            % Loop through each brick to pick and place it
            for brickIndex = 1:numBricks
                % Get the current and target (final) locations of the brick
                currentBrick = brickMatrix(brickIndex, :);  % Initial brick position
                finalBrick = finalBrickMatrix(brickIndex, :);  % Target brick position

                % Get the robot's current joint positions (arm location)
                robotLocation = r.model.getpos();

                % Compute the joint angles needed for the robot to move to the current brick's location
                currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));  % Use inverse kinematics

                % Generate a trajectory for the robot to move from its current location to the brick
                currentQPath = jtraj(robotLocation, currentBrickPath, 100);  % Joint-space trajectory

                % Animate the robot as it moves to the current brick
                for i = 1:size(currentQPath, 1)
                    % Move the robot's arm to each waypoint in the trajectory
                    r.model.animate(currentQPath(i, :));

                    % Get the current end-effector transform (position and orientation of the robot's hand)
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                    % Attach and position the left and right gripper fingers
                    % Position the left finger (-x direction relative to the end-effector)
                    baseTr1 = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
                    finger1.model.base = baseTr1;
                    finger1.model.animate([0, 0]);

                    % Position the right finger (+x direction relative to the end-effector)
                    baseTr2 = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotx(pi/2) * trotz(pi/2);
                    finger2.model.base = baseTr2;
                    finger2.model.animate([0, 0]);

                    % Update graphics to show changes in finger and robot arm position
                    drawnow();
                end

                % Simulate the gripper closing to grip the brick
                closeAngle = 0.2;  % Adjust the closing angle as necessary

                % Animate both fingers closing around the brick
                finger1.model.animate([closeAngle, closeAngle]);
                finger2.model.animate([-closeAngle, -closeAngle]);

                % Adjust the brick's position to align it between the gripper fingers
                brickOffset = transl(0, 0, -0.05);  % Adjustment for centering the brick

                % Calculate joint angles to move the brick to its final location
                finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));

                % Generate a trajectory for the robot to move the brick to its final location
                finalQPath = jtraj(currentBrickPath, finalBrickPath, 100);

                % Animate the robot as it moves the brick to the final location
                for i = 1:size(finalQPath, 1)
                    r.model.animate(finalQPath(i, :));

                    % Get and log the current end-effector transform during the move
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                    % Update gripper finger positions and move the brick along with the end-effector
                    baseTr1 = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
                    finger1.model.base = baseTr1;
                    finger1.model.animate([0, 0]);

                    baseTr2 = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(pi/2);
                    finger2.model.base = baseTr2;
                    finger2.model.animate([0, 0]);

                    % Apply the brick's updated transform to move it to the final location
                    brickTransform = endEffectorTransform * brickOffset;
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)] * brickTransform';
                    set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));

                    drawnow();  % Update the graphics to reflect the brick's movement
                end

                % Simulate releasing the brick by opening the gripper fingers
                finger1.model.animate([0, 0]);  % Open left finger
                finger2.model.animate([0, 0]);  % Open right finger
            end
        end
    end
end
