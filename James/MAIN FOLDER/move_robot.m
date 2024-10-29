classdef move_robot < handle
    properties (Constant)
        collisionThreshold = 0.05;  % Collision threshold distance
    end

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

            % Generate obstacle points for collision detection
            obstaclePoints = move_robot.generate_obstacle_point_clouds([-1.0, -0.5, 0.8; -1.3, -0.6, 0.75], 0.05);

            % Default positions for resetting on Resume
            ur3e_default_pos = r.model.getpos();
            iiwa_default_pos = r2.model.getpos();

            % Initialize GUI with persistent control
            global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
            select_brick_gui(r2, r);

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
                    move_robot.move_iiwa_with_ps4(r2, blade2, blade2Vertices);
                    ps4_control_enabled = false;
                end

                % Check and execute movements based on selections
                if movement_requested && ~estop_activated
                    if iiwa_movement_requested && ~isempty(selected_brick)
                        disp('Moving IIWA to preset positions...');
                        move_robot.move_iiwa_three_positions(r2, blade2, blade2Vertices);
                        move_robot.move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstaclePoints);
                    
                    elseif iiwa_movement_requested
                        disp('Moving only IIWA to preset positions...');
                        move_robot.move_iiwa_three_positions(r2, blade2, blade2Vertices);
                    
                    elseif ~isempty(selected_brick)
                        disp('Moving only UR3 to Brick');
                        move_robot.move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstaclePoints);
                    end

                    selected_brick = [];
                    iiwa_movement_requested = false;
                    movement_requested = false;
                end

                pause(0.1);
            end
        end

        %% Move UR3 to the selected brick using RMRC and detect collisions using point clouds
   function move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex, obstaclePoints)
    global estop_activated;
    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
    
    % Get vertices of current brick
    vertices = get(bricks{brickIndex}, 'Vertices');
    ghostVertices = vertices;  % Create ghost brick vertices (copy of original vertices)
    
    % Initial position and offset setup
    brickPos = mean(vertices);  % Center point of brick
    disp('Initial brick position (xyz):');
    disp(brickPos);
    
    % Set up initial transformation and offset calculation
    startTr = double(r.model.fkine(r.model.getpos()).T);
    endEffectorPos = startTr(1:3, 4)';
    initialOffset = brickPos - endEffectorPos;
    disp('Initial offset (brick - end effector):');
    disp(initialOffset);
    
    % Transform to brick position
    currentBrick = brickMatrix(brickIndex, :);
    brickTr = double(transl(currentBrick) * troty(pi));
    
    % First move - to brick with collision detection
    [qMatrix, ~] = controller.computeTrajectory(startTr, brickTr, 5);
    
    for i = 1:size(qMatrix, 1)
        if estop_activated, break; end
        r.model.animate(qMatrix(i, :));
        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
        endEffectorPos = endEffectorTransform(1:3, 4)';
        
        % Collision detection check
        if move_robot.is_collision_detected_point_cloud(endEffectorPos, obstaclePoints, move_robot.collisionThreshold)
            disp(['Collision detected at: ', mat2str(endEffectorPos)]);
            plot3(endEffectorPos(1), endEffectorPos(2), endEffectorPos(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
            % Continue path even after detecting a collision (no stopping or avoidance)
        end
        
        % Attach gripper and update position
        gripper.attachToEndEffector(endEffectorTransform);
        drawnow();
    end
    
    % Final move - to final position
    if ~estop_activated
        finalBrick = finalBrickMatrix(brickIndex, :);
        finalTr = double(transl(finalBrick) * troty(pi));
        
        % Compute trajectory to final position
        [qMatrix, ~] = controller.computeTrajectory(brickTr, finalTr, 5);
        
        % Move to final position and continuously update brick position
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;
            
            % Collision detection check
            endEffectorPos = endEffectorTransform(1:3, 4)';
            if move_robot.is_collision_detected_point_cloud(endEffectorPos, obstaclePoints, move_robot.collisionThreshold)
                disp(['Collision detected at: ', mat2str(endEffectorPos)]);
                plot3(endEffectorPos(1), endEffectorPos(2), endEffectorPos(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
                % Continue path even after detecting a collision
            end
            
            % Update brick position with offset
            ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
            ghostBrickPos = mean(ghostTransformedVertices(:,1:3));
            currentOffset = ghostBrickPos - endEffectorPos;

            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
            transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset;
            set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));

            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
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

        %% Function to move IIWA through three positions
        function move_iiwa_three_positions(r2, blade2, blade2Vertices)
            global estop_activated;
            iiwaPositions = [
                -1.2, -0.8, 0.76;  % Position 1
                -1.2, -0.6, 1.0;   % Position 2
                -1.2, -0.4, 1.2    % Position 3
            ];
            orientation = trotx(pi);

            for j = 1:size(iiwaPositions, 1)
                if estop_activated, break; end
                targetTransform = transl(iiwaPositions(j, :)) * orientation;
                
                targetQ = r2.model.ikcon(targetTransform);
                currentQPath = jtraj(r2.model.getpos(), targetQ, 100);

                for i = 1:size(currentQPath, 1)
                    if estop_activated, break; end
                    r2.model.animate(currentQPath(i, :));
                    endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
                    transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
                    set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
                    drawnow();
                end
            end
        end

        %% PS4 control function for the IIWA end effector
        function move_iiwa_with_ps4(r2, blade2, blade2Vertices)
            global ps4_control_enabled estop_activated resume_requested;
            joy = vrjoystick(1);
            duration = 300;
            dt = 0.15;
            Kv = 0.3;
            Kw = 0.8;
            deadZone = 0.1;

            tic;
            while toc < duration && ps4_control_enabled
                [axes, buttons, ~] = read(joy);

                if buttons(3) == 1 && ~estop_activated
                    estop_activated = true;
                    ps4_control_enabled = false;
                    resume_requested = false;
                    disp('E-Stop activated via PS4 controller.');
                    return;

                elseif buttons(2) == 1 && estop_activated
                    resume_requested = true;
                    estop_activated = false;
                    ps4_control_enabled = false;
                    disp('Resume activated via PS4 controller.');
                    pause(0.1);
                    return;
                end

                vx = Kv * (abs(axes(1)) > deadZone) * axes(1);
                vy = Kv * (abs(axes(2)) > deadZone) * axes(2);
                vz = Kv * ((buttons(5) - buttons(7)) ~= 0) * (buttons(5) - buttons(7));
                wx = Kw * (abs(axes(4)) > deadZone) * axes(4);
                wy = Kw * (abs(axes(3)) > deadZone) * axes(3);
                wz = Kw * ((buttons(6) - buttons(8)) ~= 0) * (buttons(6) - buttons(8));
                dx = [vx; vy; vz; wx; wy; wz];

                J = r2.model.jacob0(r2.model.getpos());
                dq = pinv(J) * dx;

                q = r2.model.getpos() + dq' * dt;
                r2.model.animate(q);

                endEffectorTransform = r2.model.fkine(q).T;
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

                pause(dt);
            end
            disp('Exiting PS4 control mode.');
        end
    end
end



% function activate_estop()
%     % Global variable to track the E-Stop state
%     global estop_activated;
%     estop_activated = true;  % Activate E-Stop
% 
%     % Display message
%     disp('E-Stop activated: All robot movements paused.');
% 
%     % Stop all robot movements
%     stop_robot_movement();  % This would be a custom function to halt robot actions
% end
% 
% function resume_simulation(r, r2, ur3_default_pos, iiwa_default_pos)
%     % Global variable to track the resume request state
%     global estop_activated resume_requested;
% 
%     % Check if E-Stop is currently active
%     if estop_activated
%         % Reset E-Stop and activate resume
%         resume_requested = true;
%         estop_activated = false;
% 
%         % Display message
%         disp('Resuming simulation. Robots reset to default positions.');
% 
%         % Reset robots to default positions
%         r.model.animate(ur3_default_pos);  % Animate UR3 to default position
%         r2.model.animate(iiwa_default_pos);  % Animate IIWA to default position
% 
%         % Clear the resume request
%         resume_requested = false;
%     end
% end