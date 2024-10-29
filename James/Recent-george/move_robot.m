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

            % Define collision avoidance parameters
            obstacles = [-1.0, -0.5, 0.8; -1.3, -0.6, 0.75];
            semiMajorAxis = 0.45;  
            semiMinorAxis = 0.25;  
            acceptableZThreshold = 0.4;  
            detourZOffset = 0.1;  

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
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstacles, semiMajorAxis, semiMinorAxis, acceptableZThreshold, detourZOffset);
                    
                    elseif iiwa_movement_requested
                        disp('Moving only IIWA to preset positions...');
                        move_iiwa_three_positions(r2, blade2, blade2Vertices);
                    
                    elseif ~isempty(selected_brick)
                        disp('Moving only UR3 to Brick');
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick, obstacles, semiMajorAxis, semiMinorAxis, acceptableZThreshold, detourZOffset);
                    end

                    selected_brick = [];
                    iiwa_movement_requested = false;
                    movement_requested = false;
                end

                pause(0.1);
            end
        end
    end
end

%% Function to move UR3 to the selected brick using RMRC with obstacle avoidance
function move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex, obstacles, semiMajorAxis, semiMinorAxis, acceptableZThreshold, detourZOffset)
    global estop_activated;
    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
    
    % Current position transform
    startTr = double(r.model.fkine(r.model.getpos()).T);
    
    % Transform to brick position
    currentBrick = brickMatrix(brickIndex, :);
    brickTr = double(transl(currentBrick) * troty(pi));
    
    % Move to brick using RMRC
    [qMatrix, ~] = controller.computeTrajectory(startTr, brickTr, 5);
    originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
    brickToEndEffectorOffset = eye(4);

    % Animate movement to brick with obstacle avoidance
    for i = 1:size(qMatrix, 1)
        if estop_activated, break; end
        r.model.animate(qMatrix(i, :));
        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
        gripper.attachToEndEffector(endEffectorTransform);

        % Obstacle avoidance check for gripper tip
        gripperTipPosition = endEffectorTransform(1:3, 4)';
        for j = 1:size(obstacles, 1)
            obstaclePosition = obstacles(j, :);
            relativePosition = gripperTipPosition - obstaclePosition;
            inEllipse = (relativePosition(1)^2 / semiMajorAxis^2) + (relativePosition(2)^2 / semiMinorAxis^2) <= 1;

            % Z-check for collision
            if inEllipse && abs(relativePosition(3)) < acceptableZThreshold
                disp(['Collision detected with gripper at: ', mat2str(gripperTipPosition)]);
                theta = atan2(relativePosition(2), relativePosition(1));
                detourPoint = obstaclePosition + [semiMajorAxis * cos(theta) * 2, semiMinorAxis * sin(theta) * 2, gripperTipPosition(3) + detourZOffset];
                detourPath = r.model.ikcon(transl(detourPoint) * troty(pi));
                newQPath = [jtraj(qMatrix(i, :), detourPath, 50); jtraj(detourPath, qMatrix(i+1, :), 50)];
                qMatrix = [qMatrix(1:i-1, :); newQPath];
                break;
            end
        end
        drawnow();
    end
end
