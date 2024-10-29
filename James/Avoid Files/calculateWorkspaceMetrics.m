function [workspace_volume, MX, MY, MZ] = calculateWorkspaceMetrics(r, samsize)

    %Initialises an empty matrix of size samples x3. This will store the
    %xyz pos of the end effector for however many sample size is set
    endEff_pos = zeros(samsize, 3);
    
    %Loops through the sample size, generating a random joint config for
    %the robot in each iteration and storing the corresposnding end
    %effector pos.
    for i = 1:samsize
        %generates random num between 0-1 for joints whilst being inside
        %the joint limits
        randomJointConfig = rand(1, r.model.n) .* (r.model.qlim(:,2)' - r.model.qlim(:,1)') + r.model.qlim(:,1)';
        %computes forward kinematics of joint config
        T_cell = r.model.fkine(randomJointConfig);
        T = T_cell.T;
        endEff_pos(i, :) = T(1:3, 4)';
    end

    % Calculate workspace volume using convex hull
    [~, workspace_volume] = convhull(endEff_pos(:,1), endEff_pos(:,2), endEff_pos(:,3));
    
    % Calculate maximum reach in X, Y, Z directions
    MX = max(endEff_pos(:, 1)) - min(endEff_pos(:, 1));
    MY = max(endEff_pos(:, 2)) - min(endEff_pos(:, 2));
    MZ = max(endEff_pos(:, 3)) - min(endEff_pos(:, 3));
    
    % Print out the results
    fprintf('Workspace volume: %.4f cubic meters\n', workspace_volume);
    fprintf('Reach at X: %.4f meters\n', MX);
    fprintf('Reach at Y: %.4f meters\n', MY);
    fprintf('Reach at Z: %.4f meters\n', MZ);

end
