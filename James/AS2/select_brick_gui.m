function select_brick_gui()
    global selected_brick;
    selected_brick = [];  % Initialize as empty
    
    % Define the brick options
    brick_names = {'Bread 1', 'Bread 2', 'Bread 3'}; %,'Brick 4', 'Brick 5', 'Brick 6', 'Brick 7', 'Brick 8', 'Brick 9'};
    
    % Create the GUI figure
    f = figure('Position', [100, 100, 300, 150], 'Name', 'Select Brick to Move');
    
    % Create a dropdown menu
    uicontrol('Style', 'text', 'Position', [50, 90, 200, 20], 'String', 'Select a brick to move:');
    brick_menu = uicontrol('Style', 'popupmenu', 'Position', [50, 70, 200, 20], 'String', brick_names);
    
    % Create a button to confirm selection
    uicontrol('Style', 'pushbutton', 'Position', [100, 30, 100, 30], 'String', 'Move Brick', 'Callback', @move_selected_brick);
    
    % Callback function to set the selected brick and close the GUI
    function move_selected_brick(~, ~)
        selected_brick = brick_menu.Value;  % Store the selected brick index in the global variable
        disp(['You selected ', brick_names{selected_brick}]);
        close(f);  % Close the GUI
    end
end
