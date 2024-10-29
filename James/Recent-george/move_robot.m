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

            % Define point cloud-based collision parameters
            obstaclePoints = move_robot.generate_obstacle_point_clouds([-1.0, -0.5, 0.8; -1.3, -0.6, 0.75], 0.05);
            collisionThreshold = 0.05;  % Collision threshold distance

            % Default positions for resetting on Resume
            ur3e_default_pos = r.model.getpos();
            iiwa_default_pos = r2.model.getpos();

            % Initialize GUI with persistent control
            global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
            select_brick_gui();

            % Main control loop to allow repeated GUI usage
            while true
                % Check for E-Stop
                if estop_activated
                    disp('E-Stop activated. Stopping all movements.');
                    selected_brick = [];
                    iiwa_movement_requested = false;
                    ps4_control_enabled = false;

                    while ~resume_requested
                        drawnow();
                        pause(0.1);
                    end

                    % Reset robots to default positions on Resume
                    r.model.animate(ur3e_default_pos);
                    r2.model.animate(iiwa_default_pos);
                    disp('Resuming simulation. Robots reset to default positions.');
                    estop_activated = false;
                    resume_requested = false;
                end

                % Start PS4 control if requested
                if ps4_control_enabled && ~estop_activated
                    disp('Starting PS4 control of IIWA end effector...');
                    move_iiwa_with_ps4(r2, blade2, blade2Vertices);
                    ps4_control_enabled = false;
                end

                % Execute movements based on GUI selections
                if movement_requested && ~estop_activated
                    if iiwa_movement_requested && ~isempty(selected_brick)
                        disp('Moving IIWA to preset positions...');
                        move_iiwa_three_positions(r2, blade2, blade2Vertices);
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstaclePoints, collisionThreshold);
                    
                    elseif iiwa_movement_requested
                        disp('Moving only IIWA to preset positions...');
                        move_iiwa_three_positions(r2, blade2, blade2Vertices);
                    
                    elseif ~isempty(selected_brick)
                        disp('Moving only UR3 to Brick');
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstaclePoints, collisionThreshold);
                    end

                    selected_brick = [];
                    iiwa_movement_requested = false;
                    movement_requested = false;
                end

                pause(0.1);
            end
        end

        %% Generate point clouds for obstacles
        function obstaclePoints = generate_obstacle_point_clouds(obstacleCenters, spacing)
            obstaclePoints = [];
            for i = 1:size(obstacleCenters, 1)
                center = obstacleCenters(i, :);
                [X, Y, Z] = meshgrid(center(1)-0.1:spacing:center(1)+0.1, ...
                                     center(2)-0.1:spacing:center(2)+0.1, ...
                                     center(3)-0.1:spacing:center(3)+0.1);
                points = [X(:), Y(:), Z(:)];
                obstaclePoints = [obstaclePoints; points];
            end
        end

        %% Check for collision using point cloud data
        function collisionDetected = is_collision_detected_point_cloud(position, obstaclePoints, threshold)
            distances = sqrt(sum((obstaclePoints - position).^2, 2));
            collisionDetected = any(distances < threshold);
        end
    end
end

%% Function to move UR3 to the selected brick using RMRC with point cloud-based collision detection
function move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex, obstaclePoints, collisionThreshold)
    global estop_activated;
    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
    
    % Current position transform
    startTr = double(r.model.fkine(r.model.getpos()).T);
    
    % Transform to brick position
    currentBrick = brickMatrix(brickIndex, :);
    brickTr = double(transl(currentBrick) * troty(pi));
    
    % Move to brick using RMRC with internal DLS for smoothness
    [qMatrix, ~] = controller.computeTrajectory(startTr, brickTr, 5);
    originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
    brickToEndEffectorOffset = eye(4);

    % Initial detour distance multiplier for adaptive detours
    detourMultiplier = 1;

    i = 1; % Start index for animation loop

    % Main movement loop with RMRC path recalculation upon collision
    while i <= size(qMatrix, 1) && ~estop_activated
        % Animate the robot and update gripper position
        r.model.animate(qMatrix(i, :));
        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
        gripper.attachToEndEffector(endEffectorTransform);

        % Define end-effector position for collision check
        endEffectorPosition = endEffectorTransform(1:3, 4)';

        % Check for collision with point cloud-based detection
        if move_robot.is_collision_detected_point_cloud(endEffectorPosition, obstaclePoints, collisionThreshold)
            disp(['Collision detected at end effector: ', mat2str(endEffectorPosition)]);
            collisionDetected = true;

            % Adaptive detour calculation
            theta = atan2(endEffectorPosition(2), endEffectorPosition(1));
            detourPoint = endEffectorPosition + [cos(theta) * detourMultiplier, sin(theta) * detourMultiplier, detourMultiplier];
            detourTr = transl(detourPoint) * troty(pi);
            
            % Recalculate path to detour point
            [detourQMatrix, ~] = controller.computeTrajectory(endEffectorTransform, detourTr, 3);
            
            % Concatenate detour path with remaining original path
            remainingQMatrix = qMatrix(i+1:end, :);
            qMatrix = [qMatrix(1:i, :); detourQMatrix; remainingQMatrix];
            
            % Increase detour multiplier for repeated collision avoidance
            detourMultiplier = detourMultiplier * 1.2;
            i = i + size(detourQMatrix, 1); % Move index to end of detour path
            continue; % Restart loop with new path
        end

        % Increment if no collision detected
        i = i + 1;
        drawnow();
    end

    % Final move to place the brick at the final destination
    if ~estop_activated
        finalBrick = finalBrickMatrix(brickIndex, :);
        finalTr = double(transl(finalBrick) * troty(pi));
        
        % Compute RMRC trajectory to final destination with DLS for smoothness
        [qMatrix, ~] = controller.computeTrajectory(endEffectorTransform, finalTr, 5);
        
        % Animate to the final position
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;
            brickTransform = endEffectorTransform * brickToEndEffectorOffset;
            transformedBrickVertices = [originalBrickVertices, ones(size(originalBrickVertices, 1), 1)] * brickTransform';
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end

    % Force the brick to final position if not stopped
    if ~estop_activated
        disp('Forcing brick to final position.');
        currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
        translation = finalBrick - mean(currentBrickVertices);
        set(bricks{brickIndex}, 'Vertices', currentBrickVertices + translation);
    end
end
