function select_brick_gui()
    global selected_brick;
    global isPaused;
    selected_brick = [];  % Initialize as empty
    isPaused = false;     % Initialize pause state as false
    
    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};
    
    % Create the GUI figure
    f = figure('Position', [100, 100, 300, 200], 'Name', 'Select Brick to Move');
    
    % Create a dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 140, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 120, 200, 20], 'String', brick_names);
    
    % Create a button to confirm brick selection
    uicontrol('Style', 'pushbutton', 'Position', [100, 80, 100, 30], 'String', 'Move Brick', 'Callback', @move_selected_brick);
    
    % Create a button to pause/resume the simulation
    pause_button = uicontrol('Style', 'pushbutton', 'Position', [100, 40, 100, 30], 'String', 'Pause Simulation', 'Callback', @pauseSimulation);
    
    % Callback function to set the selected brick and close the GUI
    function move_selected_brick(~, ~)
        selected_brick = brick_menu.Value;  % Store the selected brick index in the global variable
        disp(['You selected ', brick_names{selected_brick}]);
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
