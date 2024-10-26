function select_brick_gui()
    global selected_brick iiwa_movement_requested isPaused movement_requested;
    selected_brick = [];
    iiwa_movement_requested = false;
    isPaused = false;
    movement_requested = false;

    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};

    % Create the GUI figure
    f = figure('Position', [100, 100, 300, 300], 'Name', 'Select Brick and Move');

    % Dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 220, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 200, 200, 20], 'String', brick_names);

    % Button to confirm brick selection
    uicontrol('Style', 'pushbutton', 'Position', [50, 160, 200, 30], 'String', 'Select Brick', 'Callback', @select_brick);

    % Button to move IIWA
    uicontrol('Style', 'pushbutton', 'Position', [50, 120, 200, 30], 'String', 'Move IIWA', 'Callback', @request_iiwa_move);

    % "Go" button to start movement sequence
    go_button = uicontrol('Style', 'pushbutton', 'Position', [50, 80, 200, 30], 'String', 'Go', 'Enable', 'off', 'Callback', @start_movement);

    % Pause/resume button
    pause_button = uicontrol('Style', 'pushbutton', 'Position', [50, 40, 200, 30], 'String', 'Pause Simulation', 'Callback', @pauseSimulation);

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

    function enableGoButton()
        if ~isempty(selected_brick) && iiwa_movement_requested
            set(go_button, 'Enable', 'on');
        end
    end

    function start_movement(~, ~)
        movement_requested = true;
        disp('Movement sequence initiated.');
        close(f);
    end

    function pauseSimulation(~, ~)
        if isPaused
            isPaused = false;
            set(pause_button, 'String', 'Pause Simulation');
            disp('Simulation Resumed');
        else
            isPaused = true;
            set(pause_button, 'String', 'Resume Simulation');
            disp('Simulation Paused');
        end
    end
end
