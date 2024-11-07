function select_brick_gui(r2, r)
    global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
    global iiwa_xyz ur3_xyz;

    % Initialize global variables
    selected_brick = [];
    iiwa_movement_requested = false;
    movement_requested = false;
    ps4_control_enabled = false;  
    estop_activated = false;  
    resume_requested = false;  
    iiwa_xyz = r2.model.fkine(r2.model.getpos()).t;  % Set initial XYZ coordinates for IIWA
    ur3_xyz = r.model.fkine(r.model.getpos()).t;  % Set initial XYZ coordinates for UR3

    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};

    % Create the GUI figure
    f = figure('Position', [100, 100, 600, 600], 'Name', 'Control Panel');

    % Dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 540, 200, 20], 'String', 'Select a bread to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 520, 200, 20], 'String', brick_names);

    % Button to confirm brick selection
    uicontrol('Style', 'pushbutton', 'Position', [50, 480, 200, 30], 'String', 'Select Bread', 'Callback', @select_brick);

    % Button to move IIWA
    uicontrol('Style', 'pushbutton', 'Position', [50, 440, 200, 30], 'String', 'Move IIWA', 'Callback', @request_iiwa_move);

    % Button to enable PS4 control
    uicontrol('Style', 'pushbutton', 'Position', [50, 400, 200, 30], 'String', 'Enable PS4 Control', 'Callback', @enable_ps4_control);

    % Button to disable PS4 control
    uicontrol('Style', 'pushbutton', 'Position', [50, 360, 200, 30], 'String', 'Disable PS4 Control', 'Callback', @disable_ps4_control);

    % "Go" button to start movement sequence
    go_button = uicontrol('Style', 'pushbutton', 'Position', [50, 320, 200, 30], 'String', 'Go', 'Enable', 'off', 'Callback', @start_movement);

    % E-Stop button
    uicontrol('Style', 'pushbutton', 'Position', [50, 280, 200, 30], 'String', 'E-Stop', 'Callback', @activate_estop);

    % Resume button
    uicontrol('Style', 'pushbutton', 'Position', [50, 240, 200, 30], 'String', 'Reset', 'Callback', @resume_simulation);

    % IIWA End Effector XYZ Controls
    uicontrol('Style', 'text', 'Position', [300, 540, 150, 20], 'String', 'IIWA End Effector XYZ');

    uicontrol('Style', 'text', 'Position', [300, 510, 50, 20], 'String', 'X:');
    x_input_iiwa = uicontrol('Style', 'edit', 'Position', [350, 510, 100, 20], 'String', num2str(iiwa_xyz(1)), 'Callback', @update_iiwa_x);

    uicontrol('Style', 'text', 'Position', [300, 480, 50, 20], 'String', 'Y:');
    y_input_iiwa = uicontrol('Style', 'edit', 'Position', [350, 480, 100, 20], 'String', num2str(iiwa_xyz(2)), 'Callback', @update_iiwa_y);

    uicontrol('Style', 'text', 'Position', [300, 450, 50, 20], 'String', 'Z:');
    z_input_iiwa = uicontrol('Style', 'edit', 'Position', [350, 450, 100, 20], 'String', num2str(iiwa_xyz(3)), 'Callback', @update_iiwa_z);

    % UR3 End Effector XYZ Controls
    uicontrol('Style', 'text', 'Position', [300, 380, 150, 20], 'String', 'UR3 End Effector XYZ');

    uicontrol('Style', 'text', 'Position', [300, 350, 50, 20], 'String', 'X:');
    x_input_ur3 = uicontrol('Style', 'edit', 'Position', [350, 350, 100, 20], 'String', num2str(ur3_xyz(1)), 'Callback', @update_ur3_x);

    uicontrol('Style', 'text', 'Position', [300, 320, 50, 20], 'String', 'Y:');
    y_input_ur3 = uicontrol('Style', 'edit', 'Position', [350, 320, 100, 20], 'String', num2str(ur3_xyz(2)), 'Callback', @update_ur3_y);

    uicontrol('Style', 'text', 'Position', [300, 290, 50, 20], 'String', 'Z:');
    z_input_ur3 = uicontrol('Style', 'edit', 'Position', [350, 290, 100, 20], 'String', num2str(ur3_xyz(3)), 'Callback', @update_ur3_z);

    % Callback functions for GUI controls
    function select_brick(~, ~)
        selected_brick = brick_menu.Value;
        disp(['You selected ', brick_names{selected_brick}]);
        enableGoButton();
    end

    function request_iiwa_move(~, ~)
        iiwa_movement_requested = true;
        disp('IIWA movement requested');
        enableGoButton();
    end

    function enable_ps4_control(~, ~)
        ps4_control_enabled = true;
        disp('PS4 control enabled for IIWA');
    end

    function disable_ps4_control(~, ~)
        ps4_control_enabled = false;
        disp('PS4 control disabled. Returning to normal GUI mode.');
    end

    function enableGoButton()
        if (~isempty(selected_brick) || iiwa_movement_requested)
            set(go_button, 'Enable', 'on');
        end
    end

    function start_movement(~, ~)
        movement_requested = true;
        disp('Movement sequence initiated.');
    end

    function activate_estop(~, ~)
        estop_activated = true;
        disp('E-Stop activated');
    end

    function resume_simulation(~, ~)
        resume_requested = true;
        disp('Resuming simulation and resetting robots');
    end

    % IIWA Position Update Functions
    function update_iiwa_x(src, ~)
        iiwa_xyz(1) = str2double(src.String);
        update_iiwa_position();
    end

    function update_iiwa_y(src, ~)
        iiwa_xyz(2) = str2double(src.String);
        update_iiwa_position();
    end

    function update_iiwa_z(src, ~)
        iiwa_xyz(3) = str2double(src.String);
        update_iiwa_position();
    end

    function update_iiwa_position()
        target_transform = transl(iiwa_xyz);  % Translate to target XYZ position
        target_q = r2.model.ikcon(target_transform);  % Inverse kinematics for target pose
        r2.model.animate(target_q);  % Animate the IIWA model to the new position
    end

    % UR3 Position Update Functions
    function update_ur3_x(src, ~)
        ur3_xyz(1) = str2double(src.String);
        update_ur3_position();
    end

    function update_ur3_y(src, ~)
        ur3_xyz(2) = str2double(src.String);
        update_ur3_position();
    end

    function update_ur3_z(src, ~)
        ur3_xyz(3) = str2double(src.String);
        update_ur3_position();
    end

    function update_ur3_position()
        target_transform = transl(ur3_xyz);  % Translate to target XYZ position
        target_q = r.model.ikcon(target_transform);  % Inverse kinematics for target pose
        r.model.animate(target_q);  % Animate the UR3 model to the new position
    end
end
