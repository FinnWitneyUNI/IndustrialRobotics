%test gripper
classdef move_robot < handle
    %ROBOTFUNCTIONALITY 
    %class handles the robots motion and function.
    %   The code runs as follows:
    %brick locations are placed in a brick matrix, and the robot is able to
    %access those locations and using the robot arm travel towards those
    %locations.
    %The Bricks are initally placed at 0,0,0 and translated to the brick
    %locations. Then, when picked up, their position is constantly updated
    %along the robot arm and then on then finally on the table.
    
    properties
    end
        
    methods
    end

    methods (Static)
        function robot()
            %clear workspace and clear figures
            clear all;
            close all;
            clf;

                
            %populate MATLAB environment with safety equipment
            WorkSpaceEnv.Run()



            
            %Assigning initial brick locations to the matrix
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
                        
            %assigning final brick locations ot the matrix
            finalBrickMatrix = zeros(9,3);
            finalBrickMatrix(1,:) = [0.4, 0, 0.72];
            finalBrickMatrix(2,:) = [0.4, 0.2, 0.72];
            finalBrickMatrix(3,:) = [0.4, -0.2, 0.72];
            finalBrickMatrix(4,:) = [0.4, 0.2, 0.72+.033];
            finalBrickMatrix(5,:) = [0.4, 0, 0.72+.033];
            finalBrickMatrix(6,:) = [0.4, -0.2, 0.72+.033];
            finalBrickMatrix(7,:) = [0.4, 0.2, 0.72+0.066];
            finalBrickMatrix(8,:) = [0.4, 0, 0.72+0.066];
            finalBrickMatrix(9,:) = [0.4, -0.2, 0.72+0.066];
            

            % Define the number of bricks
            numBricks = 9;
            bricks = cell(numBricks, 1);
            
            % Create and store the bricks in a loop. this will assign a unique identifier to the bricks so I can call them in the future.
            % spawn in brick at 0,0,0. translate by the initial brick
            % location matrix
            for brickIndex = 1:numBricks
                bricks{brickIndex} = PlaceObject('HalfSizedRedGreenBrick.ply'); %brickMatrix(brickIndex, :)
                vertices = get(bricks{brickIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(brickMatrix(brickIndex,:))';
                set(bricks{brickIndex},'Vertices',transformedVertices(:,1:3));
            end
            
            
            %place robot model on table.
            defaultBaseTr = [1,0,0,0; 0,1,0,0; 0,0,1,0.7; 0,0,0,1];
            r = LinearUR3e(defaultBaseTr);
            r.model;
            


            
            %get initial starting robot position
            initialPosition = r.model.getpos();

            %Create a nested for loop to iterate through the robot arms
            %path and update the animation
            
            %For each brick 1-9:
            for brickIndex = 1:numBricks

                % Get the initial and final brick location
                currentBrick = brickMatrix(brickIndex, :);
                finalBrick = finalBrickMatrix(brickIndex, :);

                %set count variable to dictate how many joint angle paths
                %will be created to model the Path.
                count = 100;

                %get the robots current arm location
                robotLocation = r.model.getpos();
                
                %calculate the joint angles necessary ot traverse to the
                %chosen brick location. Brick is rotated so the Z-axis is
                %facing down here so that the robot grabs the brick from
                %the top.
                currentBrickPath = r.model.ikcon(transl(currentBrick)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and chosen brick location.
                currentQPath = jtraj(robotLocation, currentBrickPath, count);

                %for each joint step
                for i = 1:size(currentQPath, 1)
                    %animate the robots arm
                    r.model.animate(currentQPath(i, :));
                    drawnow();
                end


                
                % move to bricks final location

                %calculate the joint angles necessary to traverse to the
                %brick location. Brick is rotated so the Z-axis is
                %facing down here so that the robot places the brick from
                %the top.
                finalBrickPath = r.model.ikcon(transl(finalBrick)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and final brick location.
                finalQPath = jtraj(currentBrickPath, finalBrickPath, count);

                %for each joint step
                for i = 1:size(finalQPath, 1)

                    %animate arm
                    r.model.animate(finalQPath(i, :));

                    %calculate the end effector transform of the robot arm
                    endEffectorTransform = r.model.fkine(r.model.getpos()).T;
                    
                    %update brick vertices to the new coordinates of the
                    %robots end effector
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                    %update the bricks vertices with the transformed
                    %vertices
                    set(bricks{brickIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end
               
            end
           
            %move robot back to initial position
            count = 100;
            robotLocation = r.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
               r.model.animate(QPath(i, :));
               drawnow();
            end
            
   
        end
    end
end

