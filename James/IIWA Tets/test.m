classdef test < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            
            profile on;
            
            % Calls method run from workspaceenv to set up the environment
            % WorkSpaceEnv.Run()

            % Define the initial positions of the 9 bricks, each having xyz coordinates
            % brickMatrix = zeros(3, 3);
            % brickMatrix(1, :) = [-0.8, 0.37, 0.7];
            % brickMatrix(2, :) = [-0.7, 0.35, 0.7];
            % brickMatrix(3, :) = [-0.6, 0.35, 0.7];
            % brickMatrix(4, :) = [-0.5, 0.35, 0.7];
            % brickMatrix(5, :) = [-0.4, 0.35, 0.7];
            % brickMatrix(6, :) = [-0.3, 0.35, 0.7];
            % brickMatrix(7, :) = [-0.2, 0.35, 0.7];
            % brickMatrix(8, :) = [-0.1, 0.35, 0.7];
            % brickMatrix(9, :) = [0, 0.35, 0.7]; 

            % % Define the final positions of bricks (as a stacked wall or grid)
            % finalBrickMatrix = zeros(3, 3);
            %finalBrickMatrix(1, :) = [0.4, 0, 0.72];
            % finalBrickMatrix(2, :) = [0.4, 0.2, 0.72];
            % finalBrickMatrix(3, :) = [0.4, -0.2, 0.72];
            % % finalBrickMatrix(4, :) = [0.4, 0.2, 0.72 + 0.033];
            % % finalBrickMatrix(5, :) = [0.4, 0, 0.72 + 0.033];
            % % finalBrickMatrix(6, :) = [0.4, -0.2, 0.72 + 0.033];
            % % finalBrickMatrix(7, :) = [0.4, 0.2, 0.72 + 0.066];
            % % finalBrickMatrix(8, :) = [0.4, 0, 0.72 + 0.066];
            % % finalBrickMatrix(9, :) = [0.4, -0.2, 0.72 + 0.066];
            % 
            % % There are total 9 bricks
            % numBricks = 3;
            % bricks = cell(numBricks, 1);
            % 
            % % Load a 3D object of the brick and place it in the scene
            % for brickIndex = 1:numBricks
            %     bricks{brickIndex} = PlaceObject('bread.ply');
            %     vertices = get(bricks{brickIndex}, 'Vertices');
            %     transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
            %     set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            % end
            
            % Set up the robot model
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            % r = LinearUR3e(defaultBaseTr);
            % r.model;

            translationX = transl(1, 0, 0);
            newBaseTr = defaultBaseTr * translationX;
            i = IIWA7(newBaseTr);
            i.model;

            % % Launch the external GUI for brick selection
            % disp('Launching brick selection GUI...');
            % select_brick_gui();  % Call the external GUI script
            % 
            % % Wait for the user to select a brick (using a global variable)
            % global selected_brick;
            % selected_brick = [];  % Initialize as empty
            % 
            % % Keep checking until the user selects a brick
            % while isempty(selected_brick)
            %     pause(0.1);  % Brief pause to wait for GUI interaction
            % end
            % 
            % % Now, move the selected brick based on the user's choice
            % brickIndex = selected_brick;
            % disp(['Moving Brick ', num2str(brickIndex)]);
            % 
            % % Retrieve robot's current position
            % robotLocation = r.model.getpos();
            % 
            % % Compute joint angles to move the robot to the selected brick
            % currentBrick = brickMatrix(brickIndex, :);
            % finalBrick = finalBrickMatrix(brickIndex, :);
            % count = 100;  % Number of steps for trajectory
            % 
            % % Move to the selected brick
            % currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));
            % currentQPath = jtraj(robotLocation, currentBrickPath, count);
            % 
            % for i = 1:size(currentQPath, 1)
            %     r.model.animate(currentQPath(i, :));
            %     drawnow();
            % end
            % 
            % % Move the selected brick to its final position
            % finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));
            % finalQPath = jtraj(currentBrickPath, finalBrickPath, count);
            % 
            % % Store the original brick vertices (before the movement starts)
            % originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
            % 
            % for i = 1:size(finalQPath, 1)
            %     % Animate the robot along the calculated trajectory
            %     r.model.animate(finalQPath(i, :));
            % 
            %     % Get the current end-effector transformation matrix
            %     endEffectorTransform = double(r.model.fkine(r.model.getpos()));
            % 
            %     % Extract rotation (R) and translation (T) from the transformation matrix
            %     R = endEffectorTransform(1:3, 1:3);  % Rotation matrix
            %     T = endEffectorTransform(1:3, 4)';   % Translation vector
            % 
            %     % Apply the transformation to the brick's original vertices to move it with the end-effector
            %     transformedBrickVertices = (originalBrickVertices * R') + T;  % Transform the original vertices
            % 
            %     % Update the brick's position with the transformed vertices
            %     set(bricks{brickIndex}, 'Vertices', transformedBrickVertices);
            % 
            %     % Redraw the scene
            %     drawnow();
            % 
            %     % Display current positions for debugging
            %     disp('Current End-Effector Position:');
            %     disp(T);  % Translation vector at each step
            % 
            %     disp('Current Brick Position:');
            %     disp(mean(get(bricks{brickIndex}, 'Vertices')));  % Brick center position at each step
            % end
            % 
            % % After the robot finishes moving, force the brick to its final position
            % disp('Forcing brick to final position.');
            % 
            % % Manually set the brick's position by shifting its vertices to the final location
            % % Get the current brick vertices
            % currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
            % 
            % % Calculate the center of the brick (current position)
            % currentBrickCenter = mean(currentBrickVertices);
            % 
            % % Calculate the translation needed to move the brick to the final position
            % translation = finalBrick - currentBrickCenter;
            % 
            % % Translate the brick's vertices to the final position
            % transformedBrickVertices = currentBrickVertices + translation;
            % 
            % % Update the brick's position with the transformed vertices
            % set(bricks{brickIndex}, 'Vertices', transformedBrickVertices);
            % 
            % % Debugging: Display expected and actual positions
            % disp('Expected Final Brick Position:');
            % disp(finalBrick);
            % 
            % disp('Actual Final Brick Position after forcing:');
            % disp(mean(get(bricks{brickIndex}, 'Vertices'))); % Show final average position
        end
    end
end
