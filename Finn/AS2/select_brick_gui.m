function select_brick_gui(model, weights, initialGuess)
    global selected_brick;
    global selected_spread;  % New global variable for spread selection
    global isPaused;
    selected_brick = [];  % Initialize as empty
    selected_spread = 'Butter';  % Default to Butter
    isPaused = false;     % Initialize pause state as false
    
    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'};
    
    % Define the spread options
    spread_options = {'Butter', 'Jam'};
    
    % Create the GUI figure
    f = figure('Position', [100, 100, 300, 250], 'Name', 'Select Brick and Spread to Move');
    
    % Create a dropdown menu for brick selection
    uicontrol('Style', 'text', 'Position', [50, 180, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 160, 200, 20], 'String', brick_names);
    
    % Create a dropdown menu for spread selection
    uicontrol('Style', 'text', 'Position', [50, 140, 200, 20], 'String', 'Select spread:');
    spread_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 120, 200, 20], 'String', spread_options);
    
    % Create a button to confirm brick and spread selection
    uicontrol('Style', 'pushbutton', 'Position', [100, 80, 100, 30], 'String', 'Move Brick', 'Callback', @move_selected_brick);
    
    % Add sliders for x, y, z position control of the UR3's end effector
    uicontrol('Style', 'text', 'Position', [50, 220, 200, 20], 'String', 'Control End Effector Position');
    
    % Define the limits for the sliders based on UR3's workspace
    xMin = -0.8; xMax = 0.8;
    yMin = -0.8; yMax = 0.8;
    zMin = 0.0; zMax = 1.0;
    
    % Create sliders for x, y, z
    xSlider = uicontrol('Style', 'slider', 'Min', xMin, 'Max', xMax, 'Value', 0, 'Position', [50, 200, 200, 20]);
    ySlider = uicontrol('Style', 'slider', 'Min', yMin, 'Max', yMax, 'Value', 0, 'Position', [50, 170, 200, 20]);
    zSlider = uicontrol('Style', 'slider', 'Min', zMin, 'Max', zMax, 'Value', 0.5, 'Position', [50, 140, 200, 20]);
    
    % Add labels for the sliders
    uicontrol('Style', 'text', 'Position', [10, 200, 30, 20], 'String', 'X');
    uicontrol('Style', 'text', 'Position', [10, 170, 30, 20], 'String', 'Y');
    uicontrol('Style', 'text', 'Position', [10, 140, 30, 20], 'String', 'Z');
    
    % Callback function to update the UR3's end-effector position based on slider values
    function updateRobotPose(~, ~)
        x = get(xSlider, 'Value');
        y = get(ySlider, 'Value');
        z = get(zSlider, 'Value');
        
        % Use ikcon for SerialLink models
        endEffectorPose = transl(x, y, z);  % Create the translation matrix for the desired position
        qSol = model.ikcon(endEffectorPose, initialGuess);  % Compute the joint angles
        
        model.animate(qSol);  % Animate the robot to the new joint configuration
    end
    
    % Attach callback listeners to sliders
    addlistener(xSlider, 'Value', 'PostSet', @updateRobotPose);
    addlistener(ySlider, 'Value', 'PostSet', @updateRobotPose);
    addlistener(zSlider, 'Value', 'PostSet', @updateRobotPose);

    % Create a button to pause/resume the simulation
    pause_button = uicontrol('Style', 'pushbutton', 'Position', [100, 40, 100, 30], 'String', 'Pause Simulation', 'Callback', @pauseSimulation);
    
    % Callback function to set the selected brick and spread, and close the GUI
    function move_selected_brick(~, ~)
        selected_brick = brick_menu.Value;  % Store the selected brick index
        selected_spread = spread_menu.String{spread_menu.Value};  % Store the selected spread
        
        disp(['You selected ', brick_names{selected_brick}, ' with ', selected_spread]);
        
        % Here you can call the function that handles the actual robot motion
        % and takes into account the spread selection (Butter or Jam)
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
