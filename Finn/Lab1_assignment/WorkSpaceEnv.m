classdef WorkSpaceEnv < handle
    %ENVIRONMENTSETUP
    %This class handles the creation of the environment.
    %Models were used from both the peter corkes toolbox and online,
    %modified by meshlab and blender. This class was designed to be called
    %by the main class 'robotFunctionality'.
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

            %spawning environment
            workspace = [-5 5 -5 5 -5 3];
            hold on;
            view(3);
            axis([-2.5, 2.5, -2.5, 2.5, 0, 2]);

            %camlight illuminates the env to get a btter view
            camlight;
            %concrete textruing for floor
            surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5], [0.01,0.01;0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ...
                ,'FaceColor','texturemap');

            %place table
            PlaceObject('table.ply',[0,0,0.6]);
           

            
            %place human obj
            h = PlaceObject('3D_MAN.ply', [-1000, 0, 1000]);  % Place the human at the origin
            
            % Retrieve the vertices of the human object
            verts = get(h, 'Vertices');
            
            % Apply a scaling transformation (if needed)
            scale_factor = 0.001;  % Adjust if needed
            scaling_matrix = scale_factor * eye(3);
            scaled_verts = (scaling_matrix * verts')';  % Scale the object
            
            % Apply a rotation transformation to make the human stand up
            rotation_matrix = trotx(-pi/2);  % Rotate by -90 degrees around the x-axis (to stand up)
            
            % Convert scaled vertices to homogeneous coordinates for transformation
            homogeneous_verts = [scaled_verts, ones(size(scaled_verts, 1), 1)];
            
            % Apply the rotation matrix (note: using the 4x4 transformation matrix)
            rotated_verts = (rotation_matrix * homogeneous_verts')';  % Rotate the vertices
            
            % Update the object with rotated vertices
            set(h, 'Vertices', rotated_verts(:, 1:3));
            

           
            %place fences 
            PlaceObject('Fence_meshlab.ply',[0,1.5,0]); 
            PlaceObject('Fence_meshlab.ply',[0,-1.5,0]); 


            %place emergency stop button on the wall
            h = PlaceObject('emergencyStopWallMounted.ply',[0.7, 0.5, -0.68]);
            verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotx(pi);
            set(h,'Vertices',verts(:,1:3))
            PlaceObject('fireExtinguisher.ply',[-0.75,-0.25,0]);
 
        end
    end
end