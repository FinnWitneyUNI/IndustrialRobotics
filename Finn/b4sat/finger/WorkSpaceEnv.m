classdef WorkSpaceEnv < handle
    %Following will setup the environment.
    %it includes safety featrues such as fire extinguisher, estop and fence
    %there is a person there who is operating the robot and ready to estop
    %The estop is there if any unknown movements of the robot occur or if
    %it drops the payload etc.
    %The fire extinguisher is there to put out any electrical fires cause
    %from the robots motors overheating if being ran properly etc.
    properties
    end

    methods

        function self = WorkSpaceEnv()
            self.Run()

        end
    end

    methods (Static)

        function Run()
            close all;            
            clear all;
            clf;

            %create environment
            hold on;
            view(3);
            axis([-2.5, 2.5, -2.5, 2.5, 0, 2.75]);
            %axis off

            %camlight illuminates the env to get a btter view
            camlight;
            %concrete textruing for floor
            surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5], [0.01,0.01;0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ...
                ,'FaceColor','texturemap');

            %place table
            PlaceObject('chonkytable.ply',[0,0,0.6]);
            PlaceObject('plate.ply',[0.3,0,0.72]);
 
            % Place human object at a very far-off position for initial loading
            % h = PlaceObject('3D_MAN.ply', [-1000, 0, 1000]);
            % 
            % % Retrieve the vertices and apply scaling and rotation directly
            % scaled_and_rotated_verts = (trotx(-pi/2) * [0.001 * get(h, 'Vertices'), ones(size(get(h, 'Vertices'), 1), 1)]')';
            % 
            % % Update the object with the transformed vertices
            % set(h, 'Vertices', scaled_and_rotated_verts(:, 1:3));
            % 
            %place fences 
            %PlaceObject('fence.ply',[0,1.5,0]); 
            PlaceObject('fence.ply',[0,-1.5,0]); 
            PlaceObject('kitchen1.ply',[-2.5,2.5,0]); 

            %place emergency stop button on the wall
            h = PlaceObject('estop.ply',[0.7, 0.5, -0.68]);
            verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotx(pi);
            set(h,'Vertices',verts(:,1:3))
            PlaceObject('fireExtinguisher.ply',[-0.75,-0.25,0]);
 
        end
    end
end