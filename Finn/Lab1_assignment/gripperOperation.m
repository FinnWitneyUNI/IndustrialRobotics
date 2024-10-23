function gripperOperation(finger1, finger2, closeAngle, operation, steps)
    % Function to open or close the gripper fingers incrementally
    % Inputs:
    %   - finger1: The left finger (LinearFinger object)
    %   - finger2: The right finger (LinearFinger object)
    %   - closeAngle: The target closing angle (in radians)
    %   - operation: A string ('close' or 'open') to specify the operation
    %   - steps: Number of steps for gradual animation

    % Loop through steps to animate the fingers opening/closing
    for t = 1:steps
        if strcmp(operation, 'close')
            % Gradually close left finger
            finger1.model.animate([closeAngle * (t / steps), closeAngle * (t / steps)]);

            % Gradually close right finger
            finger2.model.animate([-closeAngle * (t / steps), -closeAngle * (t / steps)]);
        elseif strcmp(operation, 'open')
            % Gradually open left finger
            finger1.model.animate([closeAngle * (1 - t / steps), closeAngle * (1 - t / steps)]);

            % Gradually open right finger
            finger2.model.animate([-closeAngle * (1 - t / steps), -closeAngle * (1 - t / steps)]);
        end
        
        drawnow();  % Update the plot after each step
    end
end
