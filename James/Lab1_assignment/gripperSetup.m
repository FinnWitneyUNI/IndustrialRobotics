function gripperSetup(r, brickMatrix, finalBrickMatrix, gripper_finger_spacing)
    % Function to setup and animate the two-fingered gripper using the LinearFinger class
    % and simulate picking up bricks and placing them.

    % Create two instances of the LinearFinger model (assuming LinearFinger.m defines a class)
    finger1 = LinearFinger();  % Create the left finger
    finger2 = LinearFinger();  % Create the right finger

    % Define the number of bricks
    numBricks = size(brickMatrix, 1);
    bricks = cell(numBricks, 1);

    % Create and place bricks at their initial locations
    for brickIndex = 1:numBricks
        bricks{brickIndex} = PlaceObject('HalfSizedRedGreenBrick.ply');
        vertices = get(bricks{brickIndex}, 'Vertices');
        transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex,:))';
        set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
    end

    % Loop for each brick
    for brickIndex = 1:numBricks
        % Get the initial and final brick location
        currentBrick = brickMatrix(brickIndex, :);
        finalBrick = finalBrickMatrix(brickIndex, :);

        % Get the robot's current arm location
        robotLocation = r.model.getpos();

        % Calculate joint angles for the robot to reach the current brick
        currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));  % Rotate for top-gripping

        % Create joint space trajectory for the robot to move to the brick
        currentQPath = jtraj(robotLocation, currentBrickPath, 100);

        % Animate the robot to the current brick
        for i = 1:size(currentQPath, 1)
            % Animate the robot arm
            r.model.animate(currentQPath(i, :));

            % Get end effector transform
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Attach the gripper fingers to the robot's end effector
            % Left finger (-x direction)
            baseTr1 = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
            finger1.model.base = baseTr1;
            finger1.model.animate([0, 0]);

            % Right finger (+x direction)
            baseTr2 = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotx(pi/2) * trotz(pi/2);
            finger2.model.base = baseTr2;
            finger2.model.animate([0, 0]);

            drawnow();
        end

        % Simulate gripping the brick by closing the fingers
        closeAngle = 0.2;  % Adjust this value for finger closing

        % Close the fingers
        finger1.model.animate([closeAngle, closeAngle]);  % Simulate closing finger1
        finger2.model.animate([-closeAngle, -closeAngle]);  % Simulate closing finger2

        % Move the brick with the robot's end effector
        % Center the brick between the fingers
        brickOffset = transl(0, 0, -0.05);  % Adjust as needed for proper alignment

        % Now, move to the final brick location (the wall-building location)
        finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));

        % Create joint space trajectory for the robot to move to the final brick location
        finalQPath = jtraj(currentBrickPath, finalBrickPath, 100);

        % Animate the robot to the final brick location
        for i = 1:size(finalQPath, 1)
            % Animate the robot arm
            r.model.animate(finalQPath(i, :));

            % Get end effector transform
            endEffectorTransform = r.model.fkine(r.model.getpos()).T;

            % Update gripper finger positions (still attached to the end effector)
            finger1.model.base = endEffectorTransform * transl(-gripper_finger_spacing / 2, 0, 0) * trotz(-pi/2);
            finger1.model.animate([closeAngle, closeAngle]);

            finger2.model.base = endEffectorTransform * transl(gripper_finger_spacing / 2, 0, 0) * trotz(-pi/2);
            finger2.model.animate([-closeAngle, -closeAngle]);

            % Move the brick with the robot's end effector (center it between the fingers)
            brickTransform = endEffectorTransform * brickOffset;
            transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)] * brickTransform';
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));

            drawnow();
        end

        % Simulate releasing the brick by opening the fingers
        finger1.model.animate([0, 0]);  % Open finger1
        finger2.model.animate([0, 0]);  % Open finger2
    end
end