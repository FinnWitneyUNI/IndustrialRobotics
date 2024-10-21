classdef startup < handle
    % The 'startup' class serves as the entry point for initializing the robot simulation.
    % This class is designed to clear previous workspace data, set up the environment,
    % define brick positions, and run the main task of moving bricks using the robot arm.
    
    properties
        % No properties are currently defined for this class.
    end
    
    methods
        % No methods defined here as the main method is static.
    end
    
    methods (Static)
        % Static methods do not require an instance of the class to be called.
        % The 'robot' method is the main entry point for this startup class.
        
        function robot()
            % Clear the workspace and close all existing figures
            clear all;  % Clear all variables from the workspace
            close all;  % Close all open figure windows
            clf;        % Clear the current figure

            % Populate the MATLAB environment with safety equipment and initialize the workspace
            % This function (presumably from another class) sets up the environment, 
            % such as placing objects like safety fences or tables in the simulation.
            env.Run();

            % Assigning initial brick locations in 3D space into a matrix called 'brickMatrix'.
            % Each row of the matrix defines the [x, y, z] coordinates of a brick.
            % These positions represent where the bricks start on the table.
            brickMatrix = zeros(1,3);  % Initialize a 9x3 matrix for storing brick coordinates
            brickMatrix(1,:) = [-1.8, 0.4, 0.7];  % First brick position

                        
            % Assigning final brick locations (after they have been moved by the robot) 
            % into 'finalBrickMatrix'. These positions define where the robot should move each brick.
            finalBrickMatrix = zeros(1,3);  % Initialize a 9x3 matrix for final brick locations
            finalBrickMatrix(1,:) = [0.4, 0, 0.72];       % Final position for brick 1

            
            % Place the robot model on the table by defining its default base transformation.
            % The base transformation 'defaultBaseTr' places the robot at a height of 0.7 meters.
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);  % Create an instance of the 'LinearUR3e' robot model and apply the base transform
            r.model;  % Initialize the robot model

            % Example: Calculate the workspace volume of the UR3e robot by sampling random configurations.
            % numSamples = 10000;  % Number of samples for workspace calculation
            % workspace_volume = calculateWorkspaceVolume(r, numSamples);  % Calculate workspace volume
            % 
            % % Example: Calculate the maximum reach of the UR3e robot in the x, y, z planes.
            % [maxReachX, maxReachY, maxReachZ] = calculateMaxReach(r, numSamples);  % Maximum reach calculation

            % Define the spacing between the two gripper fingers, setting it to 0.08 meters.
            gripper_finger_spacing = 0.08;  % Adjusted spacing between gripper fingers

            % Call the 'runTask' function from the 'main' class to begin the process of
            % picking up bricks from their initial positions and placing them at the final positions.
            % This function handles the main task including robot movement, gripper control, and brick manipulation.
            main.runTask(r, brickMatrix, finalBrickMatrix, gripper_finger_spacing);
        end
    end
end
