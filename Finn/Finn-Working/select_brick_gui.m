function select_brick_gui()
    global selected_brick;
    global iiwa_movement_requested;
    global isPaused;
    selected_brick = [];       % Initialize as empty
    iiwa_movement_requested = false; % Initialize IIWA move request
    isPaused = false;          % Initialize pause state as false

    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};

    % Create the GUI figure
    f = figure('Position', [100, 100, 300, 250], 'Name', 'Select Brick to Move');

    % Create a dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 180, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 160, 200, 20], 'String', brick_names);

    % Create a button to confirm brick selection
    uicontrol('Style', 'pushbutton', 'Position', [50, 120, 200, 30], 'String', 'Move Brick', 'Callback', @move_selected_brick);

    % Create a button to move the IIWA robot
    uicontrol('Style', 'pushbutton', 'Position', [50, 80, 200, 30], 'String', 'Move IIWA', 'Callback', @move_iiwa);

    % Create a button to pause/resume the simulation
    pause_button = uicontrol('Style', 'pushbutton', 'Position', [50, 40, 200, 30], 'String', 'Pause Simulation', 'Callback', @pauseSimulation);

    % Callback function to set the selected brick
    function move_selected_brick(~, ~)
        selected_brick = brick_menu.Value;  % Store selected brick index
        disp(['You selected ', brick_names{selected_brick}]);
        close(f); % Close the GUI after selection
    end

    % Callback function to set the IIWA move request flag
    function move_iiwa(~, ~)
        iiwa_movement_requested = true;  % Set the global IIWA movement flag
        disp('IIWA movement requested');
        close(f); % Close the GUI after IIWA request
    end

    % Callback function for pausing and resuming the simulation
    function pauseSimulation(~, ~)
        if isPaused
            isPaused = false;  % Resume the simulation
            set(pause_button, 'String', 'Pause Simulation');  % Update button text
            disp('Simulation Resumed');
        else
            isPaused = true;   % Pause the simulation
            set(pause_button, 'String', 'Resume Simulation');  % Update button text
            disp('Simulation Paused');
        end
    end
end
