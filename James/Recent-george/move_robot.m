classdef move_robot < handle
    methods (Static)
        function robot()
            % Initialize environment and load GUI
            clf;
            profile on;
            WorkSpaceEnv.Run();

            % Define initial and final positions for UR3e bricks
            brickMatrix = [-1.35, -0.55, 0.76; -1.85, -0.4, 0.8; -1.85, -0.4, 0.85];
            finalBrickMatrix = [-0.44, -0.45, 0.76; -1.35, -0.55, 0.8; -1.35, -0.55, 1.1];
            numBricks = 3;
            bricks = cell(numBricks, 1);

            % Load bricks
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end

            % Set up UR3e and IIWA models and attach tools
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);
            r.model;
            gripper = Gripper();
            gripper.attachToEndEffector(r.model.fkine(r.model.getpos()).T);

            % Create RMRC controller for UR3e
            controller = RMRCController(r.model, 0.05);

            % Set up the IIWA7 model and attach blade
            r2 = IIWA7(defaultBaseTr * transl(-1.35, -1.2, 0));
            r2.model;
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * r2.model.fkine(r2.model.getpos()).T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Define collision detection parameters
            obstaclePoints = move_robot.generate_obstacle_point_clouds([-1.0, -0.5, 0.8; -1.3, -0.6, 0.75], 0.05);
            collisionThreshold = 0.05;  % Collision threshold distance

            % Main loop to handle robot movements
            global selected_brick;
            select_brick_gui();

            while true
                if ~isempty(selected_brick)
                    disp('Moving UR3 to Brick');
                    move_robot.move_ur3_to_brick(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstaclePoints, collisionThreshold);
                    selected_brick = [];
                end
                pause(0.1);
            end
        end

        %% Move UR3 to the selected brick and detect collisions using point clouds
        function move_ur3_to_brick(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex, obstaclePoints, collisionThreshold)
            disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
            
            % Set start and final transforms
            startTr = double(r.model.fkine(r.model.getpos()).T);
            currentBrick = brickMatrix(brickIndex, :);
            brickTr = double(transl(currentBrick) * troty(pi));

            % Compute trajectory
            [qMatrix, ~] = controller.computeTrajectory(startTr, brickTr, 5);

            % Traverse the trajectory and detect collisions
            for i = 1:size(qMatrix, 1)
                r.model.animate(qMatrix(i, :));
                currentTr = r.model.fkine(r.model.getpos()).T;
                endEffectorPosition = currentTr(1:3, 4)';

                % Check for collision using point cloud-based detection
                if move_robot.is_collision_detected_point_cloud(endEffectorPosition, obstaclePoints, collisionThreshold)
                    % Log collision coordinates
                    disp(['Collision detected at: ', mat2str(endEffectorPosition)]);
                    
                    % Mark collision point in the 3D simulation environment
                    plot3(endEffectorPosition(1), endEffectorPosition(2), endEffectorPosition(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
                end

                % Attach gripper and update brick position
                gripper.attachToEndEffector(currentTr);
                move_robot.update_brick_position(bricks, brickIndex, currentTr, gripper, brickMatrix);
                drawnow();
            end

            % Finalize brick position
            move_robot.finalize_brick_position(bricks, brickIndex, finalBrickMatrix);
        end

        %% Generate point clouds for obstacles
        function obstaclePoints = generate_obstacle_point_clouds(obstacleCenters, spacing)
            obstaclePoints = [];
            % Loop through each obstacle and create point cloud
            for i = 1:size(obstacleCenters, 1)
                center = obstacleCenters(i, :);
                % Generate points around the center using a cubic grid pattern
                [X, Y, Z] = meshgrid(center(1)-0.1:spacing:center(1)+0.1, ...
                                     center(2)-0.1:spacing:center(2)+0.1, ...
                                     center(3)-0.1:spacing:center(3)+0.1);
                points = [X(:), Y(:), Z(:)];
                obstaclePoints = [obstaclePoints; points];
            end
        end

        %% Check for collision using point cloud data
        function collisionDetected = is_collision_detected_point_cloud(position, obstaclePoints, threshold)
            % Calculate distances from end-effector position to all obstacle points
            distances = sqrt(sum((obstaclePoints - position).^2, 2));
            % Detect collision if any distance is below the threshold
            collisionDetected = any(distances < threshold);
        end

        %% Finalize brick position at the target
        function finalize_brick_position(bricks, brickIndex, finalBrickMatrix)
            finalBrick = finalBrickMatrix(brickIndex, :);
            currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
            translation = finalBrick - mean(currentBrickVertices);
            set(bricks{brickIndex}, 'Vertices', currentBrickVertices + translation);
            disp('Brick position finalized.');
        end

        %% Helper function to update brick position based on the end-effector transform
        function update_brick_position(bricks, brickIndex, endEffectorTransform, gripper, brickMatrix)
            % Get the original brick vertices
            originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
            
            % Define the transformation matrix for the brick's initial position
            brickTransform = endEffectorTransform * transl(brickMatrix(brickIndex, :)) * [eye(3), [0;0;0]; 0 0 0 1];
            
            % Transform vertices
            transformedBrickVertices = [originalBrickVertices, ones(size(originalBrickVertices, 1), 1)] * brickTransform';
            
            % Update brick position by setting transformed vertices
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
        end
    end
end
