function [maxReachX, maxReachY, maxReachZ] = calculateMaxReach(r, numSamples)
    % Function to calculate the robot's maximum reach in the X, Y, and Z planes
    % r - The robot model (SerialLink)
    % numSamples - The number of random configurations to sample

    % Initialize matrix to store end effector positions
    ee_positions = zeros(numSamples, 3);

    % Loop over the number of samples
    for i = 1:numSamples
        % Generate a random joint configuration within the joint limits
        randomJointConfig = rand(1, r.model.n) .* (r.model.qlim(:,2)' - r.model.qlim(:,1)') + r.model.qlim(:,1)';

        % Calculate the end effector position for this joint configuration
        T_cell = r.model.fkine(randomJointConfig);  % fkine output is a cell array
        
        % Extract the actual transformation matrix from the cell array
        T = T_cell.T;

        % Extract the position (x, y, z) from the transformation matrix
        ee_positions(i, :) = T(1:3, 4)';  % End effector position in X, Y, Z
    end

    % Calculate the maximum and minimum reaches in X, Y, and Z planes
    maxReachX = max(ee_positions(:, 1)) - min(ee_positions(:, 1));
    maxReachY = max(ee_positions(:, 2)) - min(ee_positions(:, 2));
    maxReachZ = max(ee_positions(:, 3)) - min(ee_positions(:, 3));

    % Display the maximum reach values
    fprintf('Maximum reach in X direction: %.4f meters\n', maxReachX);
    fprintf('Maximum reach in Y direction: %.4f meters\n', maxReachY);
    fprintf('Maximum reach in Z direction: %.4f meters\n', maxReachZ);
end
