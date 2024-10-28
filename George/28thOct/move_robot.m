classdef move_robot < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            
            profile on;
            
            % Calls method run from workspaceenv to set up the environment
            WorkSpaceEnv.Run();

            % Define the initial positions of the bricks

            brickMatrix = zeros(3, 3);
            brickMatrix(1, :) = [-1.35, -0.55, 0.76];
            brickMatrix(2, :) = [-1.85, -0.4, 0.8];
            brickMatrix(3, :) = [-1.85, -0.4, 0.85];
            % Define the final positions of bricks
            finalBrickMatrix = zeros(3, 3);
            finalBrickMatrix(1, :) = [-0.44, -0.45, 0.76];
            finalBrickMatrix(2, :) = [-1.35, -0.55, 1.1];
            finalBrickMatrix(3, :) = [-1.35, -0.55, 1.1];

            % Number of bricks
            numBricks = 3;
            bricks = cell(numBricks, 1);
            
            % Load a 3D object of the brick and place it in the scene
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end
            
            % Set up the UR3e robot model
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);  % UR3e robot
            r.model;


%             % ** Create and attach the gripper **
             gripper = Gripper();  % Create a new Gripper object
             ur3eEndEffectorTransform = r.model.fkine(r.model.getpos()).T;  % Get the UR3e end-effector transform
             gripper.attachToEndEffector(ur3eEndEffectorTransform);  % Attach the gripper to the end-effector

            % Set up the IIWA7 robot model
            translationX = transl(-1.35, -1.2, 0);
            newBaseTr = defaultBaseTr * translationX;
            r2 = IIWA7(newBaseTr);  % IIWA7 robot
            r2.model;

            % Load the blade2 object and attach it to the IIWA7's end-effector
            blade2 = PlaceObject('blade2.ply');  % Load the blade2 end-effector model
            blade2Vertices = get(blade2, 'Vertices');  % Store the vertices for transformation

            % **Attach blade to IIWA7 end-effector immediately**
            iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());  % Get IIWA7's end-effector transformation matrix
            % Apply the transformation to the blade vertices
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Launch the external GUI for brick selection
            disp('Launching brick selection GUI...');
            select_brick_gui();  % Call the external GUI script

            % Wait for the user to select a brick (using a global variable)
            global selected_brick;
            selected_brick = [];  % Initialize as empty
            
            % Keep checking until the user selects a brick
            while isempty(selected_brick)
                    iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());  

                    % Apply the transformation to the blade vertices
                    transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
                    set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

                    % Redraw the scene
                    drawnow();

                pause(0.1);  % Brief pause to wait for GUI interaction
            end
            
            % Now, move the selected brick based on the user's choice
            brickIndex = selected_brick;
            disp(['Moving Brick ', num2str(brickIndex)]);

            % Retrieve robot's current position
            robotLocation = r.model.getpos();
            
            % Compute joint angles to move the robot to the selected brick
            currentBrick = brickMatrix(brickIndex, :);
            finalBrick = finalBrickMatrix(brickIndex, :);
            count = 100;  % Number of steps for trajectory
            
            % Move to the selected brick
            currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));
            currentQPath = jtraj(robotLocation, currentBrickPath, count);

            % Prepare the original brick vertices
            originalBrickVertices = get(bricks{brickIndex}, 'Vertices');  % Get the original vertices
            
            % Initialize brick attachment variables
            brickToEndEffectorOffset = eye(4);  % Default to identity initially

            % Animate the UR3e to approach the brick
            for i = 1:size(currentQPath, 1)
                % Animate both robots
                r.model.animate(currentQPath(i, :));  % Animate UR3e robot
                % r2.model.animate(zeros(1, 7));  % Keep IIWA7 in default position (no movement for now)

                % Get the end-effector transformation of UR3e
                endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                % ** Update gripper position during animation **
                gripper.attachToEndEffector(endEffectorTransform);  % Update the gripper's position

                % **Brick Pickup Logic**
                % When the robot reaches the brick, calculate the offset between the brick and end-effector
                if i == size(currentQPath, 1)
                    % Get the brick's current transformation matrix
                    brickTransform = transl(currentBrick);  
                    % Calculate the offset between the brick and the end-effector
                    brickToEndEffectorOffset = inv(endEffectorTransform) * brickTransform;  
                end

                drawnow();
            end

            % Now move to the final brick location
            finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));
            finalQPath = jtraj(currentBrickPath, finalBrickPath, count);

            % Animate the movement while keeping the brick attached to the end-effector
            for i = 1:size(finalQPath, 1)
                % Animate both robots
                r.model.animate(finalQPath(i, :));  % Animate UR3e robot
                % r2.model.animate(zeros(1, 7));  % Keep IIWA7 stationary (or add movement if needed)

                % Get the current end-effector transformation
                endEffectorTransform = r.model.fkine(r.model.getpos()).T;

                % **Update Brick Position Based on End-Effector**
                % Apply the calculated offset to the brick's position relative to the end-effector
                brickTransform = endEffectorTransform * brickToEndEffectorOffset;  % Recalculate brick transform based on end-effector
                transformedBrickVertices = [originalBrickVertices, ones(size(originalBrickVertices, 1), 1)] * brickTransform';
                set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));  % Update brick position

                % ** Update gripper position during animation **
                gripper.attachToEndEffector(endEffectorTransform);  % Update the gripper's position

                % Attach blade2 to IIWA7 end-effector
                iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());  % Get IIWA7's end-effector position
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

                drawnow();
            end
            while true
                pause(0.5);
                
            end
                

            % After the robot finishes moving, force the brick to its final position
            disp('Forcing brick to final position.');
            
            % Manually set the brick's position by shifting its vertices to the final location
            currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
            currentBrickCenter = mean(currentBrickVertices);
            translation = finalBrick - currentBrickCenter;
            transformedBrickVertices = currentBrickVertices + translation;
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices);
            
            % Debugging: Display expected and actual positions
            disp('Expected Final Brick Position:');
            disp(finalBrick);
            
            disp('Actual Final Brick Position after forcing:');
            disp(mean(get(bricks{brickIndex}, 'Vertices'))); % Show final average position
        end
    end
end

