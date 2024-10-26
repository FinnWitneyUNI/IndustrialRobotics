classdef move_robot < handle
    methods (Static)
        function robot()
            % Initialize environment and load GUI
            clf;
            profile on;
            WorkSpaceEnv.Run();

            % Define initial and final positions for UR3e bricks
            brickMatrix = [-1.35, -0.55, 0.76; -1.85, -0.4, 0.8; -1.85, -0.4, 0.85];
            finalBrickMatrix = [-0.44, -0.45, 0.76; -1.35, -0.55, 0.8; -1.35, -0.55, 1.1];
            numBricks = 3;
            bricks = cell(numBricks, 1);

            % Load bricks
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end

            % Set up UR3e and IIWA models and attach tools
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);
            r.model;
            gripper = Gripper();
            gripper.attachToEndEffector(r.model.fkine(r.model.getpos()).T);

            % Set up the IIWA7 model and attach blade
            r2 = IIWA7(defaultBaseTr * transl(-1.35, -1.2, 0));
            r2.model;
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * r2.model.fkine(r2.model.getpos()).T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Main control loop to allow repeated GUI usage
            global selected_brick iiwa_movement_requested movement_requested;
            while true
                % Launch GUI for user to select options
                selected_brick = [];
                iiwa_movement_requested = false;
                movement_requested = false;
                select_brick_gui();

                % Wait until "Go" button is pressed
                while ~movement_requested
                    drawnow();
                    pause(0.1);
                end

                % Execute IIWA movement first
                if iiwa_movement_requested
                    disp('Moving IIWA to preset positions...');
                    move_iiwa_two_positions(r2, blade2, blade2Vertices);
                end

                % Follow up with UR3e movement
                if ~isempty(selected_brick)
                    brickIndex = selected_brick;
                    disp(['Moving UR3e to Brick ', num2str(brickIndex)]);
                    
                    % UR3e movement to selected brick and final location
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
                        drawnow();
                    end

                    % Force brick to final position
                    disp('Forcing brick to final position.');
                    currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
                    translation = finalBrick - mean(currentBrickVertices);
                    set(bricks{brickIndex}, 'Vertices', currentBrickVertices + translation);
                end

                % Reset flags for the next cycle
                selected_brick = [];
                iiwa_movement_requested = false;
                movement_requested = false;
            end
        end
    end
end

function move_iiwa_two_positions(r2, blade2, blade2Vertices)
    % Define two preset positions for the IIWA with explicit SE3 objects
    positions = {SE3(-1.2, -0.8, 0.76) * SE3.Rx(pi/2) * SE3.Ry(pi/4), ...
                 SE3(-1.2, -0.8, 1.0) * SE3.Rx(pi/2) * SE3.Ry(pi/4)};
    
    for j = 1:length(positions)
        targetQ = r2.model.ikcon(positions{j}.T);
        currentQPath = jtraj(r2.model.getpos(), targetQ, 100);

        for i = 1:size(currentQPath, 1)
            r2.model.animate(currentQPath(i, :));
            endEffectorTransform = r2.model.fkine(r2.model.getpos()).T;
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * endEffectorTransform';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
            drawnow();
        end
    end
end
