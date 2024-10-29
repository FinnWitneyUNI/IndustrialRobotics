classdef move_robot < handle
    methods (Static)
        function robot()
            % Clear workspace and initialize environment
            clear all;
            close all;
            clf;
            
            profile on;
            
            % Calls method run from workspaceenv to set up the environment
            WorkSpaceEnv.Run();

            % Define the initial positions of the bricks
            brickMatrix = [-1.35, -0.55, 0.76; -1.85, -0.4, 0.8; -1.85, -0.4, 0.85];
            finalBrickMatrix = [-0.44, -0.45, 0.76; -1.35, -0.55, 1.1; -1.35, -0.55, 1.1];
            numBricks = 3;
            bricks = cell(numBricks, 1);
            
            % Load a 3D object of the brick and place it in the scene
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('WholemealBread.ply');
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(brickMatrix(brickIndex, :))';
                set(bricks{brickIndex}, 'Vertices', transformedVertices(:, 1:3));
            end
            
            % Set up the UR3e robot model
            defaultBaseTr = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.74; 0, 0, 0, 1];
            r = LinearUR3e(defaultBaseTr);  % UR3e robot
            r.model;

            % Attach the gripper to UR3e's end-effector
            gripper = Gripper();
            ur3eEndEffectorTransform = r.model.fkine(r.model.getpos()).T;
            gripper.attachToEndEffector(ur3eEndEffectorTransform);

            % Gripper length from the end-effector (adjust to match actual length)
            gripperLength = 0.1;

            % Set up the IIWA7 robot model
            translationX = transl(-1.35, -1.2, 0);
            newBaseTr = defaultBaseTr * translationX;
            r2 = IIWA7(newBaseTr);  % IIWA7 robot
            r2.model;

            % Load and attach blade2 to IIWA7's end-effector
            blade2 = PlaceObject('blade2.ply');
            blade2Vertices = get(blade2, 'Vertices');

            iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());
            transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
            set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));

            % Launch the external GUI for brick selection
            disp('Launching brick selection GUI...');
            select_brick_gui();

            global selected_brick;
            selected_brick = [];
            
            % Wait for user to select a brick
            while isempty(selected_brick)
                iiwaEndEffectorTransform = r2.model.fkine(r2.model.getpos());
                transformedBlade2Vertices = [blade2Vertices, ones(size(blade2Vertices, 1), 1)] * iiwaEndEffectorTransform.T';
                set(blade2, 'Vertices', transformedBlade2Vertices(:, 1:3));
                drawnow();
                pause(0.1);
            end
            
            % Set up path planning for selected brick
            brickIndex = selected_brick;
            robotLocation = r.model.getpos();
            currentBrick = brickMatrix(brickIndex, :);
            finalBrick = finalBrickMatrix(brickIndex, :);
            count = 100;

            % Define elliptical safety zones for obstacles and platform
            obstacles = [-1.0, -0.5, 0.8; -1.3, -0.6, 0.75];
            platformPosition = [-1.2, -0.5, 0.76]; % Center position of the raised platform
            semiMajorAxis = 0.45;  % Slightly reduced X-axis range for obstacles
            semiMinorAxis = 0.25;  % Slightly reduced Y-axis range for obstacles
            platformSemiMajor = 0.65; % Slightly larger X range for platform
            platformSemiMinor = 0.45; % Slightly larger Y range for platform
            acceptableZThreshold = 0.4;  % Increased threshold for Z-coordinate discrepancy
            detourZOffset = 0.1;  % Z-offset to lift detour points above the platform level

            % Visualize elliptical safety zones and platform avoidance zone
            figure; hold on;
            for j = 1:size(obstacles, 1)
                obstaclePosition = obstacles(j, :);
                
                % Plot elliptical zone for each obstacle
                theta = linspace(0, 2 * pi, 100);
                ellipseX = obstaclePosition(1) + semiMajorAxis * cos(theta);
                ellipseY = obstaclePosition(2) + semiMinorAxis * sin(theta);
                plot3(ellipseX, ellipseY, obstaclePosition(3) * ones(size(theta)), 'r--', 'LineWidth', 1.5);
                
                % Debugging output
                disp(['Obstacle at position: ', mat2str(obstaclePosition)]);
            end
            
            % Plot the platform avoidance zone as an ellipse
            theta = linspace(0, 2 * pi, 100);
            platformEllipseX = platformPosition(1) + platformSemiMajor * cos(theta);
            platformEllipseY = platformPosition(2) + platformSemiMinor * sin(theta);
            plot3(platformEllipseX, platformEllipseY, platformPosition(3) * ones(size(theta)), 'b--', 'LineWidth', 1.5); % Blue dashed for platform

            hold off;

            % Move to the selected brick with elliptical collision avoidance
            currentBrickPath = r.model.ikcon(transl(currentBrick) * troty(pi));
            currentQPath = jtraj(robotLocation, currentBrickPath, count);
            
            i = 1;  % Start loop index

            while i <= size(currentQPath, 1)
                recalculatePath = false;

                % Compute the end-effector transformation
                endEffectorTransform = double(r.model.fkine(currentQPath(i, :)));
                if isequal(size(endEffectorTransform), [4, 4])
                    endEffectorPosition = endEffectorTransform(1:3, 4)';
                    % Calculate gripper tip position (end-effector + gripper length along z-axis)
                    gripperTipPosition = endEffectorPosition + gripperLength * endEffectorTransform(1:3, 3)';
                else
                    disp('Error: Invalid end-effector transformation matrix size.');
                    return;
                end

                % Check for collision with elliptical safety zones at both end-effector and gripper tip positions
                for j = 1:size(obstacles, 1)
                    % Transform coordinates for end-effector and gripper tip
                    obstaclePosition = obstacles(j, :);
                    relativePositionEE = endEffectorPosition - obstaclePosition;
                    relativePositionTip = gripperTipPosition - obstaclePosition;

                    % Check ellipse condition for end-effector and gripper tip
                    inEllipseEE = (relativePositionEE(1)^2 / semiMajorAxis^2) + ...
                                  (relativePositionEE(2)^2 / semiMinorAxis^2) <= 1;
                    inEllipseTip = (relativePositionTip(1)^2 / semiMajorAxis^2) + ...
                                   (relativePositionTip(2)^2 / semiMinorAxis^2) <= 1;

                    % Additional Z-check for collision
                    if (inEllipseEE || inEllipseTip) && ...
                       (abs(relativePositionEE(3)) < acceptableZThreshold || abs(relativePositionTip(3)) < acceptableZThreshold)
                        plot3(gripperTipPosition(1), gripperTipPosition(2), gripperTipPosition(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
                        disp(['Collision detected with gripper at: ', mat2str(gripperTipPosition)]);

                        % Adjust the detour point, factoring in the gripper tip position
                        theta = atan2(relativePositionEE(2), relativePositionEE(1));
                        detourPoint = obstaclePosition + [semiMajorAxis * cos(theta) * 2, ...
                                                          semiMinorAxis * sin(theta) * 2, ...
                                                          gripperTipPosition(3) + detourZOffset];
                        plot3(detourPoint(1), detourPoint(2), detourPoint(3), 'gx', 'MarkerSize', 10); % Visualize detour point

                        detourPath = r.model.ikcon(transl(detourPoint) * troty(pi));
                        finalPath = r.model.ikcon(transl(finalBrick) * troty(pi));

                        detourQPath = jtraj(currentQPath(i, :), detourPath, count / 2);
                        finalQPath = jtraj(detourPath, finalPath, count / 2);
                        newQPath = [currentQPath(1:i-1, :); detourQPath; finalQPath];

                        recalculatePath = true;
                        currentQPath = newQPath;
                        i = 1;  % Reset to start of new path
                        break;
                    end
                end

                if recalculatePath
                    continue;
                end

                % Move along the path
                r.model.animate(currentQPath(i, :));
                drawnow();
                pause(0.01);
                i = i + 1;
            end

            % Post-movement adjustments
            disp('Forcing brick to final position.');
            currentBrickVertices = get(bricks{brickIndex}, 'Vertices');
            translation = finalBrick - mean(currentBrickVertices);
            transformedBrickVertices = currentBrickVertices + translation;
            set(bricks{brickIndex}, 'Vertices', transformedBrickVertices);

            disp('Expected Final Brick Position:');
            disp(finalBrick);
            disp('Actual Final Brick Position:');
            disp(mean(get(bricks{brickIndex}, 'Vertices')));
        end
    end
end
