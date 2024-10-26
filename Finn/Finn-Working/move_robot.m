classdef move_robot < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            profile on;
            WorkSpaceEnv.Run();

            % Define the initial and final positions of the bricks for UR3e
            brickMatrix = [-1.35, -0.55, 0.76; -1.85, -0.4, 0.8; -1.85, -0.4, 0.85];
            finalBrickMatrix = [-0.44, -0.45, 0.76; -1.35, -0.55, 1.1; -1.35, -0.55, 1.1];
            numBricks = 3;
            bricks = cell(numBricks, 1);

            % Load and place the bricks in the scene
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end

            % Set up the UR3e robot model and gripper
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);
            r.model;
            gripper = Gripper();
            ur3eEndEffectorTransform = r.model.fkine(r.model.getpos()).T;
            gripper.attachToEndEffector(ur3eEndEffectorTransform);

            % Set up the IIWA7 robot model and attach blade
            translationX = transl(-1.35, -1.2, 0);
            r2 = IIWA7(defaultBaseTr * translationX);
            r2.model;
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');

            % Attach blade to IIWA7 end-effector
            iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Main loop for continuous selection and movement
            global selected_brick;
            global iiwa_movement_requested; % New flag for IIWA movement request
            while true
                % Launch the GUI and wait for user selection
                disp('Launching brick selection GUI...');
                select_brick_gui();
                selected_brick = [];
                iiwa_movement_requested = false; % Reset IIWA request flag each loop

                while isempty(selected_brick) && ~iiwa_movement_requested
                    % Keep checking for the selected brick or IIWA request
                    drawnow();
                    pause(0.1);
                end

                % Check selected option to move either UR3e or IIWA
                if iiwa_movement_requested
                    disp('Moving IIWA to preset positions...');
                    move_iiwa_two_positions(r2, blade2, blade2Vertices);
                elseif ~isempty(selected_brick)
                    % Existing UR3e brick movement code here
                    brickIndex = selected_brick;
                    disp(['Moving Brick ', num2str(brickIndex)]);

                    % Retrieve UR3e's current position
                    robotLocation = r.model.getpos();
                    currentBrick = brickMatrix(brickIndex, :);
                    finalBrick = finalBrickMatrix(brickIndex, :);
                    count = 100;

                    % Move UR3e to the selected brick
                    currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));
                    currentQPath = jtraj(robotLocation, currentBrickPath, count);
                    originalBrickVertices = get(bricks{brickIndex}, 'Vertices');
                    brickToEndEffectorOffset = eye(4);

                    % Animate UR3e to approach the brick
                    for i = 1:size(currentQPath, 1)
                        r.model.animate(currentQPath(i, :));
                        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
                        gripper.attachToEndEffector(endEffectorTransform);

                        if i == size(currentQPath, 1)
                            brickTransform = transl(currentBrick);
                            brickToEndEffectorOffset = inv(endEffectorTransform) * brickTransform;
                        end
                        drawnow();
                    end

                    % Move to final brick location
                    finalBrickPath = r.model.ikcon(transl(finalBrick) * troty(pi));
                    finalQPath = jtraj(currentBrickPath, finalBrickPath, count);
                    for i = 1:size(finalQPath, 1)
                        r.model.animate(finalQPath(i, :));
                        endEffectorTransform = r.model.fkine(r.model.getpos()).T;
                        brickTransform = endEffectorTransform * brickToEndEffectorOffset;
                        transformedBrickVertices = [originalBrickVertices, ones(size(originalBrickVertices, 1), 1)] * brickTransform';
                        set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                        gripper.attachToEndEffector(endEffectorTransform);

                        % Update blade position on IIWA7
                        iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());
                        transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
                        set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

                        drawnow();
                    end

                    % Force brick to final position
                    disp('Forcing brick to final position.');
                    currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
                    currentBrickCenter = mean(currentBrickVertices);
                    translation = finalBrick - currentBrickCenter;
                    transformedBrickVertices = currentBrickVertices + translation;
                    set(bricks{brickIndex}, 'Vertices', transformedBrickVertices);

                    disp('Expected Final Brick Position:');
                    disp(finalBrick);
                    disp('Actual Final Brick Position after forcing:');
                    disp(mean(get(bricks{brickIndex}, 'Vertices')));
                end
            end
        end
    end
end

function move_iiwa_two_positions(r2, blade2, blade2Vertices)
    % Define two preset positions for the IIWA with explicit SE3 objects
    positions = {SE3(-1.2, -0.8, 0.76) * SE3.Rx(pi/2) * SE3.Ry(pi/4), ...
                 SE3(-1.2, -0.8, 1.0) * SE3.Rx(pi/2) * SE3.Ry(pi/4)};
    
    for j = 1:length(positions)
        try
            % Verify transformation matrix is of class SE3
            assert(isa(positions{j}, 'SE3'), 'Transformation matrix is not SE3');

            % Calculate target joint angles
            targetQ = r2.model.ikcon(positions{j}.T);  % Pass SE3 matrix to ikcon
            currentQPath = jtraj(r2.model.getpos(), targetQ, 100);

            for i = 1:size(currentQPath, 1)
                r2.model.animate(currentQPath(i, :));
                endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
                drawnow();
            end
        catch ME
            warning("Transformation matrix failed. Reason: %s", ME.message);
        end
    end
end