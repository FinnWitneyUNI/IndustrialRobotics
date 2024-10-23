function workspace_volume = calculateWorkspaceVolume(r, numSamples)
    % Function to calculate the approximate workspace volume of a robot
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

        % Ensure the result is a valid 4x4 transformation matrix
        if size(T,1) == 4 && size(T,2) == 4
            % Extract the position (x, y, z) from the transformation matrix
            ee_positions(i, :) = T(1:3, 4)';  % Extract the position from the transformation matrix
        else
            error('Unexpected transformation matrix size. Check the fkine output.');
        end
    end

    % Use the Convex Hull algorithm to estimate the workspace volume
    [~, volume] = convhull(ee_positions(:,1), ee_positions(:,2), ee_positions(:,3));

    % Output the workspace volume
    workspace_volume = volume;
    fprintf('Approximate workspace volume: %.4f cubic meters\n', workspace_volume);
end
