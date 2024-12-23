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

            % Set up the IIWA7 model and attach blade
            r2 = IIWA7(defaultBaseTr * transl(-1.35, -1.2, 0));
            r2.model;
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * r2.model.fkine(r2.model.getpos()).T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Default positions for resetting on Resume
            ur3e_default_pos = r.model.getpos();
            iiwa_default_pos = r2.model.getpos();

            % Initialize GUI with persistent control
            global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
            select_brick_gui();  % Keep GUI open

            % Main control loop to allow repeated GUI usage
            while true
                % Check for E-Stop
                if estop_activated
                    disp('E-Stop activated. Stopping all movements.');
                    % Reset flags and pause until resume is requested
                    selected_brick = [];
                    iiwa_movement_requested = false;
                    ps4_control_enabled = false;

                    while ~resume_requested
                        drawnow(); % Wait for Resume button
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
                    ps4_control_enabled = false; % Reset flag after control ends
                end

                % Check and execute movements based on selections and Go button press
                if movement_requested && ~estop_activated
                    % If both IIWA and UR3 are selected, move IIWA first, then UR3
                    if iiwa_movement_requested && ~isempty(selected_brick)
                        disp('Moving IIWA to preset positions...');
                        move_iiwa_three_positions(r2, blade2, blade2Vertices);

                        % After IIWA completes, move UR3 to brick
                        move_ur3_to_brick(r, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick);
                    
                    % If only IIWA is selected
                    elseif iiwa_movement_requested
                        disp('Moving only IIWA to preset positions...');
                        move_iiwa_three_positions(r2, blade2, blade2Vertices);
                    
                    % If only UR3 is selected
                    elseif ~isempty(selected_brick)
                        disp('Moving only UR3 to Brick');
                        move_ur3_to_brick(r, gripper, bricks, brickMatrix, finalBrickMatrix, selected_brick);
                    end

                    % Reset selections and movement request for the next cycle
                    selected_brick = [];
                    iiwa_movement_requested = false;
                    movement_requested = false;
                end

                pause(0.1); % Small pause to avoid excessive CPU use
            end
        end
    end
end

% Function to move UR3 to the selected brick
function move_ur3_to_brick(r, gripper, bricks, brickMatrix, finalBrickMatrix, brickIndex)
    global estop_activated;
    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
    robotLocation = r.model.getpos();
    currentBrick = brickMatrix(brickIndex, :);
    finalBrick = finalBrickMatrix(brickIndex, :);
    count = 100;

    % Move UR3e to the selected brick
    currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));
    currentQPath = jtraj(robotLocation, currentBrickPath, count);
    originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
    brickToEndEffectorOffset = eye(4);

    % Animate UR3e to approach the brick
    for i = 1:size(currentQPath, 1)
        if estop_activated, break; end  % Check for E-Stop
        r.model.animate(currentQPath(i, :));
        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
        gripper.attachToEndEffector(endEffectorTransform);
        if i == size(currentQPath, 1)
            brickTransform = transl(currentBrick);
            brickToEndEffectorOffset = inv(endEffectorTransform) * brickTransform;
        end
        drawnow();
    end

    % Move to final brick location
    if ~estop_activated
        finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));
        finalQPath = jtraj(currentBrickPath, finalBrickPath, count);
        for i = 1:size(finalQPath, 1)
            if estop_activated, break; end  % Check for E-Stop
            r.model.animate(finalQPath(i, :));
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;
            brickTransform = endEffectorTransform * brickToEndEffectorOffset;
            transformedBrickVertices = [originalBrickVertices, ones(size(originalBrickVertices, 1), 1)] * brickTransform';
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
            gripper.attachToEndEffector(endEffectorTransform);
            drawnow();
        end
    end

    % Force brick to final position if no E-Stop
    if ~estop_activated
        disp('Forcing brick to final position.');
        currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
        translation = finalBrick - mean(currentBrickVertices);
        set(bricks{brickIndex}, 'Vertices', currentBrickVertices + translation);
    end
end


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

% Updated PS4 control function with added debugging for E-Stop and Resume
function move_iiwa_with_ps4(r2, blade2, blade2Vertices)
    global ps4_control_enabled estop_activated resume_requested;  % Ensure these variables are accessible
    joy = vrjoystick(1); % Initialize joystick; may need to change ID if multiple joysticks
    duration = 300;  % Duration of control session
    dt = 0.15;       % Time step for updates
    Kv = 0.3;        % Linear velocity gain
    Kw = 0.8;        % Angular velocity gain
    deadZone = 0.1;  % Dead zone threshold to prevent unintended drift

    % Start PS4 control loop
    tic;
    while toc < duration && ps4_control_enabled
        % Read joystick input
        [axes, buttons, ~] = read(joy);

        % Check buttons for E-Stop and Resume
        if buttons(2) == 1  % Button 2 for E-Stop
            estop_activated = true;
            ps4_control_enabled = false;  % Exit PS4 control on E-Stop
            disp('E-Stop activated via PS4 controller.');
            return;  % Exit function to handle E-Stop globally
        elseif buttons(3) == 1  % Button 3 for Resume
            resume_requested = true;
            estop_activated = false;  % Clear the E-Stop if resume is requested
            disp('Resume activated via PS4 controller.');
            pause(0.1);  % Small delay to debounce the button
            return;  % Exit function to re-enable movement globally
        end

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