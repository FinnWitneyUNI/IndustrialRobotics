%% This is our groups testing Environment setup script

classdef env < handle
    properties
    end

    methods

        function self = env()
            self.Run()

        end
    end

    methods (Static)

        function Run()
            close all;            
            clear all;
            clf;

            %spawning environment
            workspace = [-5 5 -5 5 -5 3];
            hold on;
            view(3);
            axis([-5, 5, -5, 5, 0, 3]);

            %camlight illuminates the env to get a btter view
            camlight;
            %concrete textruing for floor
            surf([-5,-5;5,5],[-5,5;-5,5], [0.01,0.01;0.01,0.01] ...
                ,'CData',imread('checkerboard.jpg') ...
                ,'FaceColor','texturemap');

            %place table and scale to appropriate size
            %place table
            h = PlaceObject('table.ply',[0, 0, 10]);
            % Retrieve the vertices of the human object
            verts = get(h, 'Vertices');
            
            % Apply a scaling transformation (if needed)
            scale_factor = 0.0015;  % Adjust if needed
            scaling_matrix = scale_factor * eye(3);
            scaled_verts = (scaling_matrix * verts')';  % Scale the object
                      
            % Convert scaled vertices to homogeneous coordinates for transformation
            homogeneous_verts = [scaled_verts, ones(size(scaled_verts, 1), 1)];
                       
            % Update the object 
            set(h, 'Vertices', homogeneous_verts(:, 1:3));
            
            
            %place button on table
            %place emergency stop button on the wall
            %PlaceObject('button.ply',[1, -1, -1]);

            %place emergency stop button on the wall
            h = PlaceObject('estop.ply',[1.25, 0.89, -1]);
            verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotx(pi);
            set(h,'Vertices',verts(:,1:3))
 
        end
    end
end