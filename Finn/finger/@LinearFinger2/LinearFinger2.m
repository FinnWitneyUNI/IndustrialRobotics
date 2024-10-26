classdef LinearFinger2 < RobotBaseClass
    % The LinearFinger class defines a robotic finger model that inherits 
    % properties and methods from the base class "RobotBaseClass". This 
    % class creates a simple 2-link robot, which can act as a linear finger.

    properties(Access = public)
        plyFileNameStem = 'LinearFinger';  % The name stem for the .ply file used for visualization (if any)
    end
    
    methods
        %% Constructor Function
        % This function is called when an instance of LinearFinger is created.
        % It initializes the model, sets its base transformation, and applies 
        % colors or visualization settings.
        function self = LinearFinger2(baseTr)
            % Create the model of the robot (finger)
            self.CreateModel();
            
            % If no base transformation is provided, default it to identity transformation.
            if nargin < 1			
                baseTr = transl(0,0,0);  % Default base transformation is the identity matrix (no transformation)
            end
            
            % Apply the given base transformation to the robot's base.
            % This allows for positioning the robot in the world frame.
            self.model.base =  self.model.base.T * baseTr;
            
            % Apply coloring and plot settings for visualization
            self.PlotAndColourRobot();         
        end

        %% Create the robot model
        % This method defines the kinematic structure of the robot (finger) 
        % by defining the links and joints.
        function CreateModel(self)   
            % Define the first link with its Denavit-Hartenberg (DH) parameters:
            % 'd' - offset along z-axis, 'a' - link length, 'alpha' - twist angle, 
            % 'qlim' - joint angle limits, 'offset' - offset for joint angle.
            link(1) = Link('d', 0, 'a', 0.047, 'alpha', 0, 'qlim', deg2rad([-10 25]), 'offset', deg2rad(30)); 
            
            % Define the second link with its DH parameters.
            link(2) = Link('d', 0, 'a', 0.04, 'alpha', 0, 'qlim', deg2rad([0 0.01]), 'offset', deg2rad(30));
            
            % Create a SerialLink robot model with the defined links and assign it a name.
            self.model = SerialLink(link, 'name', self.plyFileNameStem);  % The name will be 'LinearFinger'
            
            % Plot options to customize the robot visualization:
            % 'noshadow' - removes shadow, 'noarrow' - no joint arrows, 
            % 'noshading' - flat shading, 'nowrist' - no wrist markers,
            % 'nojaxes' - no axes drawn.
            self.model.plotopt = {'noshadow', 'noarrow', 'noshading', 'nowrist', 'nojaxes'};
        end
     
    end
end
