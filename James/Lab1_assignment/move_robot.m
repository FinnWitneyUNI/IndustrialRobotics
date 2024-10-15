classdef move_robot < handle
    % Main robot motion class.
    
    properties
    end
    
    methods
    end
    
    methods (Static)
        function robot()
            % Clear workspace and close all figures
            clear all;
            close all;
            clf;

            % Populate MATLAB environment with safety equipment
            WorkSpaceEnv.Run();

            
            % Assigning initial brick locations to the matrix
            brickMatrix = zeros(9,3);
            brickMatrix(1,:) = [-0.8, 0.42, 0.7];
            brickMatrix(2,:) = [-0.7, 0.4, 0.7];
            brickMatrix(3,:) = [-0.6, 0.4, 0.7];
            brickMatrix(4,:) = [-0.5, 0.4, 0.7];
            brickMatrix(5,:) = [-0.4, 0.4, 0.7];
            brickMatrix(6,:) = [-0.3, 0.4, 0.7];
            brickMatrix(7,:) = [-0.2, 0.38, 0.7];
            brickMatrix(8,:) = [-0.1, 0.4, 0.7];
            brickMatrix(9,:) = [0, 0.4, 0.7]; 
                        
            % Assigning final brick locations to the matrix
            finalBrickMatrix = zeros(9,3);
            finalBrickMatrix(1,:) = [0.4, 0, 0.72];
            finalBrickMatrix(2,:) = [0.4, 0.2, 0.72];
            finalBrickMatrix(3,:) = [0.4, -0.2, 0.72];
            finalBrickMatrix(4,:) = [0.4, 0.2, 0.72 + .033];
            finalBrickMatrix(5,:) = [0.4, 0, 0.72 + .033];
            finalBrickMatrix(6,:) = [0.4, -0.2, 0.72 + .033];
            finalBrickMatrix(7,:) = [0.4, 0.2, 0.72 + 0.066];
            finalBrickMatrix(8,:) = [0.4, 0, 0.72 + 0.066];
            finalBrickMatrix(9,:) = [0.4, -0.2, 0.72 + 0.066];

            % Place robot model on table
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.7; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);
            r.model;

            % Example of calculating the workspace volume of the UR3e robot
            %numSamples = 10000;  % Number of random configurations to sample
            %workspace_volume = calculateWorkspaceVolume(r, numSamples);

            % Example of calculating the maximum reach of the UR3e robot
            %numSamples = 10000;  % Number of random configurations to sample
            %[maxReachX, maxReachY, maxReachZ] = calculateMaxReach(r, numSamples);

            % Define gripper spacing between fingers
            gripper_finger_spacing = 0.08;  % Adjusted spacing

            % Call the gripperSetup function to handle the gripper setup and animation
            gripperSetup(r, brickMatrix, finalBrickMatrix, gripper_finger_spacing);


        end
    end
end

