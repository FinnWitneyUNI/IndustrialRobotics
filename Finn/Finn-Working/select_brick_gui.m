function select_brick_gui()
    global selected_brick iiwa_movement_requested movement_requested ps4_control_enabled estop_activated resume_requested;
    selected_brick = [];
    iiwa_movement_requested = false;
    movement_requested = false;
    ps4_control_enabled = false;  
    estop_activated = false;  
    resume_requested = false;  

    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};

    % Create the GUI figure without CloseRequestFcn
    f = figure('Position', [100, 100, 300, 460], 'Name', 'Control Panel');

    % Dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 380, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 360, 200, 20], 'String', brick_names);

    % Button to confirm brick selection
    uicontrol('Style', 'pushbutton', 'Position', [50, 320, 200, 30], 'String', 'Select Brick', 'Callback', @select_brick);

    % Button to move IIWA
    uicontrol('Style', 'pushbutton', 'Position', [50, 280, 200, 30], 'String', 'Move IIWA', 'Callback', @request_iiwa_move);

    % Button to enable PS4 control
    uicontrol('Style', 'pushbutton', 'Position', [50, 240, 200, 30], 'String', 'Enable PS4 Control', 'Callback', @enable_ps4_control);

    % Button to disable PS4 control
    uicontrol('Style', 'pushbutton', 'Position', [50, 200, 200, 30], 'String', 'Disable PS4 Control', 'Callback', @disable_ps4_control);

    % "Go" button to start movement sequence
    go_button = uicontrol('Style', 'pushbutton', 'Position', [50, 160, 200, 30], 'String', 'Go', 'Enable', 'off', 'Callback', @start_movement);

    % E-Stop button
    uicontrol('Style', 'pushbutton', 'Position', [50, 120, 200, 30], 'String', 'E-Stop', 'Callback', @activate_estop);

    % Resume button
    uicontrol('Style', 'pushbutton', 'Position', [50, 80, 200, 30], 'String', 'Resume', 'Callback', @resume_simulation);

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
        ps4_control_enabled = false;  % This will signal PS4 control loop to exit
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
end
