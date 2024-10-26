classdef move_robot < handle
%class defines as move_robot
%To run the project, tpye ///// move_robot.robot ////// into command window.
    methods (Static)
        function robot()
            %clear stuff
            clear all;
            close all;
            clf;
            
            profile on;
                
            %calls method run from workspaceenv to set up the envr
            env.Run()

            %The matrices store the position of the briks both before and
            %after the robot moves them.
            %Defines the initial positions of 9 bricks each having xyz coor
            brickMatrix = zeros(9,3);
            brickMatrix(1,:) = [-1, 0.35, 0.7];
            % brickMatrix(2,:) = [-0.7, 0.35, 0.7];
            % brickMatrix(3,:) = [-0.6, 0.35, 0.7];
            % brickMatrix(4,:) = [-0.5, 0.35, 0.7];
            % brickMatrix(5,:) = [-0.4, 0.35, 0.7];
            % brickMatrix(6,:) = [-0.3, 0.35, 0.7];
            % brickMatrix(7,:) = [-0.2, 0.35, 0.7];
            % brickMatrix(8,:) = [-0.1, 0.35, 0.7];
            % brickMatrix(9,:) = [0, 0.35, 0.7]; 
                        
            %Defines the final pos where the robot will place the bricks
            %setting up as a stacked wall as asked in doucment
            finalBrickMatrix = zeros(9,3);
            finalBrickMatrix(1,:) = [0.1, 0, 0.72];
            % finalBrickMatrix(2,:) = [0.4, 0.2, 0.72];
            % finalBrickMatrix(3,:) = [0.4, -0.2, 0.72];
            % finalBrickMatrix(4,:) = [0.4, 0.2, 0.72+.033];
            % finalBrickMatrix(5,:) = [0.4, 0, 0.72+.033];
            % finalBrickMatrix(6,:) = [0.4, -0.2, 0.72+.033];
            % finalBrickMatrix(7,:) = [0.4, 0.2, 0.72+0.066];
            % finalBrickMatrix(8,:) = [0.4, 0, 0.72+0.066];
            % finalBrickMatrix(9,:) = [0.4, -0.2, 0.72+0.066];
            % 
            %Setting up as a 3x3 grid also asked in document
            % finalBrickMatrix = zeros(9,3);
            % finalBrickMatrix(1,:) = [0.45,0.2,0.72];
            % finalBrickMatrix(2,:) = [0.48,0,0.72];
            % finalBrickMatrix(3,:) = [0.45,-0.2,0.72];
            % 
            % finalBrickMatrix(4,:) = [0.45-0.06,0.2,0.72];
            % finalBrickMatrix(5,:) = [0.30,-0.05,0.72];
            % finalBrickMatrix(6,:) = [0.45-0.06,-0.2,0.72];
            % 
            % finalBrickMatrix(7,:) = [0.4-0.1,0.2,0.72];
            % finalBrickMatrix(8,:) = [0.39,0.01,0.72];
            % finalBrickMatrix(9,:) = [0.4-0.1,-0.2,0.72];
            

            %there are total 9 bricks
            numBricks = 1;
            bricks = cell(numBricks, 1);
            
            % loads a 3d object of the brick and places it in the scene at
            % the specified positon of the brickmatrix. 
            %Each bricks positon is adjusted to its correct starting
            %locations using the transl() function which creates the
            %transformation matrix.
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('HalfSizedRedGreenBrick.ply'); %brickMatrix(brickIndex, :)
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(brickMatrix(brickIndex,:))';
                set(bricks{brickIndex},'Vertices',transformedVertices(:,1:3));
            end
            
            
            %defaultBaseTr defines the base transform matrix for the robot
            %positioning it in 3d space.
            %LinearUR3e creates model and places at base transformation
            defaultBaseTr = [1,0,0,0; 0,1,0,0; 0,0,1,1.08; 0,0,0,1];
            r = LinearUR3e(defaultBaseTr);
            r.model;


            %Retrieves robot current pos and stroes in initialpostion.
            %will be used once all bricks have been placed
            initialPosition = r.model.getpos();

            %This loop iterates over all of the 9 bricks one by one
            for brickIndex = 1:numBricks
                currentBrick = brickMatrix(brickIndex, :);
                finalBrick = finalBrickMatrix(brickIndex, :);
                %this can be changed as a result of profile tool
                count = 100;
                robotLocation = r.model.getpos();
                
                %computes joint angles (inverse kine) needed to move the
                %robots end effector to the brick pos.
                %generates it rotated z axis down using troty(pi) so brick
                %can be grabbed from above
                %Where to move
                currentBrickPath = r.model.ikcon(transl(currentBrick)*troty(pi));

                %Generates a trajectory in joint space between the robots
                %current pos and the calculated pos for picking up brick
                %How to move
                currentQPath = jtraj(robotLocation, currentBrickPath, count);

                %moves the robot along the calculated trajectory step by
                %step
                for i = 1:size(currentQPath, 1)
                    r.model.animate(currentQPath(i, :));
                    % Get and log the end effector transform
                    endEffectorTransform = r.model.fkine(r.model.getpos());
                    disp('End Effector Transform (Picking up):');
                    disp(endEffectorTransform);
                    drawnow();
                end

                %The robot calcs the join angles required to move the brick
                %back to its final location and then it animates the robot
                %following this trajectory
                finalBrickPath = r.model.ikcon(transl(finalBrick)*troty(pi));
                finalQPath = jtraj(currentBrickPath, finalBrickPath, count);

                %moves the robot along the calculated trajectory step by
                %step
                for i = 1:size(finalQPath, 1)
                    r.model.animate(finalQPath(i, :));
                    %transform calc
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;
                    %log transform
                    disp('End Effector Transform (Placing):');
                    disp(endEffectorTransform);
                    %The pos of the brick is updated as the robots end
                    %effector moves, making it look like robot is carrying
                    %brick
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';
                    set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end
               
            end
            %calls the reach and volume to then display in cmd wnd after
            [workspace_volume, MX, MY, MZ] = calculateWorkspaceMetrics(r, 1000);


            %After all bricks are moved, robot returns to intial pos
            count = 200;
            robotLocation = r.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
               r.model.animate(QPath(i, :));
               drawnow();
            end

            profile off;
            profile viewer;
            
   
        end
    end
end

