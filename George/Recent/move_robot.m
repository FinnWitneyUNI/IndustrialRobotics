classdef move_robot < handle
    methods (Static)
        function robot()
            % Initialize environment and load GUI
            clf;
            profile on;
            WorkSpaceEnv.Run();

            % Define initial and final positions for UR3e bricks
            brickMatrix = [-1.35, -0.55, 0.76; -1.85, -0.4, 0.8; -1.85, -0.4, 0.85];
            finalBrickMatrix = [-0.44, -0.45, 0.76; -1.35, -0.55, 0.75; -1.35, -0.55, 1.1];
            finalLaneBrickMatrix = [-0.44, -0.45, 0.76; -0.44, -0.45, 0.76; -1.35, -0.55, 1.1];
            numBricks = 3;
            bricks = cell(numBricks, 1);

            % Load bricks
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end
            
            jamInitialPos = [-0.947,-0.647,0.8];  % Initial position for jam
            jamFinalPos = [-1.35, -0.55, 0.88];    % Final position for jam
            jam = PlaceObject('Jam.ply');         % Make sure you have a Jam.ply file
            jamVertices = get(jam, 'Vertices');
            transformedJamVertices = [jamVertices, ones(size(jamVertices, 1), 1)] * transl(jamInitialPos)';
            set(jam, 'Vertices', transformedJamVertices(:, 1:3));
            
            
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
            
            % Set initial bent configuration
            initial_q = [pi/2, pi/8, 0, -pi/2, 0, pi/3, 0];  % Adjust these angles as needed
            r2.model.animate(initial_q);
            
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * r2.model.fkine(r2.model.getpos()).T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Default positions for resetting on Resume
            ur3e_default_pos = r.model.getpos();
            iiwa_default_pos = r2.model.getpos();

            % Initialize GUI with persistent control
            global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
            select_brick_gui(r2, r);  % Keep GUI open

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

                % Check and execute movements based on selections
                if movement_requested && ~estop_activated
                    if iiwa_movement_requested && ~isempty(selected_brick)
                        disp('Moving IIWA to preset positions...');
                         
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick);                        
                        
                        spread_jam(r2, blade2, blade2Vertices, jam, jamVertices, jamInitialPos, jamFinalPos);
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, finalBrickMatrix, finalLaneBrickMatrix, selected_brick);

                    elseif iiwa_movement_requested
                        disp('Moving only IIWA to preset positions...');
                        spread_jam(r2, blade2, blade2Vertices, jam, jamVertices, jamInitialPos, jamFinalPos);
                        
                    elseif ~isempty(selected_brick)
                        disp('Moving only UR3 to Brick');
                        move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick);
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
%% Function to move UR3 to the selected brick using RMRC
function move_ur3_to_brick_rmrc(r, controller, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex)
    global estop_activated;
    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
    
    % Get vertices of current brick
    vertices = get(bricks{brickIndex}, 'Vertices');
    % Create ghost brick vertices (copy of original vertices)
    ghostVertices = vertices;
    
    % Debug: Show initial brick position
    brickPos = mean(vertices);  % Center point of brick
    disp('Initial brick position (xyz):');
    disp(brickPos);
    
    % Current position transform
    startTr = double(r.model.fkine(r.model.getpos()).T);
    
    % Debug: Show initial end effector position
    endEffectorPos = startTr(1:3, 4)';  % Extract xyz position from transform
    disp('Initial end effector position (xyz):');
    disp(endEffectorPos);
    
    % Calculate initial offset
    initialOffset = brickPos - endEffectorPos;
    disp('Initial offset (brick - end effector):');
    disp(initialOffset);
    
    % Transform to brick position
    currentBrick = brickMatrix(brickIndex, :);
    brickTr = double(transl(currentBrick) * troty(pi));
    
    % Move to brick using RMRC
    [qMatrix, ~] = controller.computeTrajectory(startTr, brickTr, 5);

    % First movement - to brick
    for i = 1:size(qMatrix, 1)
        if estop_activated, break; end
        r.model.animate(qMatrix(i, :));
        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
        
        if mod(i, 10) == 0  % Debug every 10 steps to avoid console spam
            endEffectorPos = endEffectorTransform(1:3, 4)';
            disp('Current end effector position during approach:');
            disp(endEffectorPos);
        end
        
        gripper.attachToEndEffector(endEffectorTransform);
        drawnow();
    end
% --------above cutting board
if ~estop_activated
        % Transform to final position
        currentBrick = brickMatrix(brickIndex, :);
        brickTr = double(transl(currentBrick) * troty(pi));
        retractPos = currentBrick + [0 0 0.1];
        retractTr = double(transl(retractPos) * trotx(pi));

        % Compute trajectory for retraction
        [qMatrix, ~] = controller.computeTrajectory(brickTr, retractTr, 5);
        % Second movement - continuously update brick position
        
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Calculate ghost brick position (not displayed)
            ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
            % Get ghost brick position for offset calculation
            ghostBrickPos = mean(ghostTransformedVertices(:,1:3));
            endEffectorPos = endEffectorTransform(1:3, 4)';
            currentOffset = ghostBrickPos - endEffectorPos;

            
            % Update brick position
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
            % Add offset to all vertices of the brick
            transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset;
            set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));


            
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end
%------
% --------above lanes
if ~estop_activated
        % Transform to final position
        finalBrick = finalBrickMatrix(brickIndex, :);
        finalTr = double(transl(finalBrick) * troty(pi));
        retractPos = finalBrick + [0 0 0.1];
        retractTr2 = double(transl(retractPos) * trotx(pi));

        % Compute trajectory for retraction
        [qMatrix, ~] = controller.computeTrajectory(retractTr, retractTr2, 5);
        % Second movement - continuously update brick position
        
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Calculate ghost brick position (not displayed)
            ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
            % Get ghost brick position for offset calculation
            ghostBrickPos = mean(ghostTransformedVertices(:,1:3));
            endEffectorPos = endEffectorTransform(1:3, 4)';
            currentOffset = ghostBrickPos - endEffectorPos;

            
            % Update brick position
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
            % Add offset to all vertices of the brick
            transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset;
            set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));


            
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end
%------ down to lane
    if ~estop_activated
        % Transform to final position
        finalBrick = finalBrickMatrix(brickIndex, :);
        finalTr = double(transl(finalBrick) * troty(pi));
        
        % Compute trajectory to final position
        [qMatrix, ~] = controller.computeTrajectory(retractTr2, finalTr, 5);
        % Second movement - continuously update brick position
        
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Calculate ghost brick position (not displayed)
            ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
            % Get ghost brick position for offset calculation
            ghostBrickPos = mean(ghostTransformedVertices(:,1:3));
            endEffectorPos = endEffectorTransform(1:3, 4)';
            currentOffset = ghostBrickPos - endEffectorPos;

            
            % Update brick position
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
            % Add offset to all vertices of the brick
            transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset;
            set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));


            
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end
        % LAST MOVEMENT retract from bread
    if ~estop_activated
        % Calculate retraction position (10cm up from final position)
        retractPos = finalBrick + [0 0 0.1];
        retractTr3 = double(transl(retractPos) * trotx(pi));

        % Compute trajectory for retraction
        [qMatrix, ~] = controller.computeTrajectory(finalTr, retractTr3, 5);

        % Final movement - retract from jam
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r.model.animate(qMatrix(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Update only blade position, leaving Bread in place
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end
end
%% IIWA SPREAD
function spread_jam(r2, blade2, blade2Vertices, jam, jamVertices, jamInitialPos, jamFinalPos)
    global estop_activated;
    disp('Moving IIWA7 to spread jam');
    
    % Get current vertices
    vertices = get(jam, 'Vertices');
    % Create ghost vertices for offset calculation
    ghostVertices = vertices;
    
    % Current position transform
    startTr = double(r2.model.fkine(r2.model.getpos()).T);
    
    % Transform to initial jam position with rotation
    jamInitialBlade = jamInitialPos + [0 0 0.1]
    jamTr = double(transl(jamInitialBlade) * trotx(pi));
    
    % Create RMRC controller for IIWA7
    controller = RMRCController(r2.model, 0.05);
    
    % Move to jam using RMRC
    [qMatrix, ~] = controller.computeTrajectory(startTr, jamTr, 5);
    % First movement - to jam
    for i = 1:size(qMatrix, 1)
        if estop_activated, break; end
        r2.model.animate(qMatrix(i, :));
        endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
        
        % Update blade position
        transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
        set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
        drawnow();
    end
            % SECOND MOVEMENT ABOVE JAM 
            % Current position transform
            startTr = double(r2.model.fkine(r2.model.getpos()).T);

            % Transform to initial jam position slightly above
            jamRaised = jamInitialPos + [0 0 0.2];
            jamTr = double(transl(jamRaised) * trotx(pi));

            % Move to jam using RMRC
            [qMatrix, ~] = controller.computeTrajectory(startTr, jamTr, 5);
            
            for i = 1:size(qMatrix, 1)
                if estop_activated, break; end
                r2.model.animate(qMatrix(i, :));
                endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
                
                ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
                ghostJamPos = mean(ghostTransformedVertices(:,1:3));
                endEffectorPos = endEffectorTransform(1:3, 4)';
                currentOffset = ghostJamPos - endEffectorPos;

                % Update jam position
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
                transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset - [0 0 0.1];
                set(jam, 'Vertices', transformedVertices(:, 1:3));
                
                % Update blade position
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
                drawnow();
            end
    
    
    if ~estop_activated
        % Transform to final position
        finalTr = double(transl(jamFinalPos) * trotx(pi));
        
        % Compute trajectory to final position using RMRC
        [qMatrix, ~] = controller.computeTrajectory(jamTr, finalTr, 5);
        
        % THIRD MOVEMENT - move jam to final position
        for i = 1:size(qMatrix, 1)
            if estop_activated, break; end
            r2.model.animate(qMatrix(i, :));
            endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
            % Calculate ghost jam position
            ghostTransformedVertices = [ghostVertices, ones(size(ghostVertices, 1), 1)] * endEffectorTransform';
            ghostJamPos = mean(ghostTransformedVertices(:,1:3));
            endEffectorPos = endEffectorTransform(1:3, 4)';
            currentOffset = ghostJamPos - endEffectorPos;
            
            % Update jam position
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * endEffectorTransform';
            transformedVertices(:,1:3) = transformedVertices(:,1:3) - currentOffset - [0 0 0.1];
            set(jam, 'Vertices', transformedVertices(:, 1:3));
            
            % Update blade position
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
            drawnow();
        end
        
        
        % FOURTH MOVEMENT retract from bread
        if ~estop_activated
            % Calculate retraction position (10cm up from final position)
            retractPos = jamFinalPos + [0 0 0.4];
            retractTr = double(transl(retractPos) * trotx(pi));
            
            % Compute trajectory for retraction
            [qMatrix, ~] = controller.computeTrajectory(finalTr, retractTr, 5);
            
            % Final movement - retract from jam
            for i = 1:size(qMatrix, 1)
                if estop_activated, break; end
                r2.model.animate(qMatrix(i, :));
                endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
                
                % Update only blade position, leaving jam in place
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
                drawnow();
            end
        end
    end
end



%% Function to move IIWA through three positions
function move_iiwa_three_positions(r2, blade2, blade2Vertices)
    global estop_activated;
    % Define three target positions for the IIWA
    iiwaPositions = [
        -1.2, -0.8, 0.76;  % Position 1
        -1.2, -0.6, 1.0;   % Position 2
        -1.2, -0.4, 1.2    % Position 3
    ];
    orientation = trotx(pi);  % 180-degree rotation around X-axis for downward orientation

    % Loop through each position
    for j = 1:size(iiwaPositions, 1)
        if estop_activated, break; end  % Check E-Stop at each position
        % Combine position with downward orientation
        targetTransform = transl(iiwaPositions(j, :)) * orientation;
        
        % Calculate target joint angles for each position with orientation
        targetQ = r2.model.ikcon(targetTransform);
        currentQPath = jtraj(r2.model.getpos(), targetQ, 100);

        % Animate movement to the target position
        for i = 1:size(currentQPath, 1)
            if estop_activated, break; end  % Check E-Stop within path
            r2.model.animate(currentQPath(i, :));
            endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
            drawnow();
        end
    end
end

% PS4 control function for the IIWA end effector
function move_iiwa_with_ps4(r2, blade2, blade2Vertices)
    global ps4_control_enabled estop_activated;  % Ensure these variables are accessible
    joy = vrjoystick(1); % Initialize joystick; may need to change ID if multiple joysticks
    duration = 300;  % Duration of control session
    dt = 0.15;       % Time step for updates
    Kv = 0.3;        % Linear velocity gain
    Kw = 0.8;        % Angular velocity gain
    deadZone = 0.1;  % Dead zone threshold to prevent unintended drift

    % Start PS4 control loop
    tic;
    while toc < duration && ps4_control_enabled && ~estop_activated
        % Read joystick input
        [axes, buttons, ~] = read(joy);
        
        % Apply dead zone filtering to each axis
        vx = Kv * (abs(axes(1)) > deadZone) * axes(1);
        vy = Kv * (abs(axes(2)) > deadZone) * axes(2);
        vz = Kv * ((buttons(5) - buttons(7)) ~= 0) * (buttons(5) - buttons(7));
        wx = Kw * (abs(axes(4)) > deadZone) * axes(4);
        wy = Kw * (abs(axes(3)) > deadZone) * axes(3);
        wz = Kw * ((buttons(6) - buttons(8)) ~= 0) * (buttons(6) - buttons(8));
        dx = [vx; vy; vz; wx; wy; wz];  % Combined velocity vector

        % Calculate joint velocity using Damped Least-Squares with pseudoinverse
        J = r2.model.jacob0(r2.model.getpos());
        dq = pinv(J) * dx;

        % Update joint angles based on joint velocities
        q = r2.model.getpos() + dq' * dt;
        r2.model.animate(q);

        % Update end effector (blade) position
        endEffectorTransform = r2.model.fkine(q).T;
        transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
        set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

        % Wait for next time step
        pause(dt);
    end
    disp('Exiting PS4 control mode.');
end