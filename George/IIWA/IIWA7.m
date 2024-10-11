classdef IIWA7 < RobotBaseClass
    %% IIWA7 UR3e on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'IIWA';
    end
    
    methods
%% Define robot Function 
        function self = IIWA7(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR5 model mounted on a linear rail
            link(1) = Link('d', 0.360, 'a', 0,       'alpha', pi/2, 'qlim', deg2rad([-170, 170]), 'offset', 0);   % Joint 1
            link(2) = Link('d', 0,     'a', 0,       'alpha', -pi/2, 'qlim', deg2rad([-120, 120]), 'offset', pi/2);  % Joint 2
            link(3) = Link('d', 0.420, 'a', 0,       'alpha', pi/2, 'qlim', deg2rad([-170, 170]), 'offset', 0);   % Joint 3
            link(4) = Link('d', 0,     'a', 0,       'alpha', -pi/2, 'qlim', deg2rad([-120, 120]), 'offset', 0);   % Joint 4
            link(5) = Link('d', 0.400, 'a', 0,       'alpha', pi/2, 'qlim', deg2rad([-170, 170]), 'offset', 0);   % Joint 5
            link(6) = Link('d', 0,     'a', 0,       'alpha', -pi/2, 'qlim', deg2rad([-120, 120]), 'offset', 0);   % Joint 6
            link(7) = Link('d', 0.126, 'a', 0,       'alpha', 0, 'qlim', deg2rad([-175, 175]), 'offset', 0);   % Joint 7
            
%             % Incorporate joint limits
%             link(1).qlim = [-0.8 -0.01];
%             link(2).qlim = [-360 360]*pi/180;
%             link(3).qlim = [-90 90]*pi/180;
%             link(4).qlim = [-170 170]*pi/180;
%             link(5).qlim = [-360 360]*pi/180;
%             link(6).qlim = [-360 360]*pi/180;
%             link(7).qlim = [-360 360]*pi/180;
%         
%             link(3).offset = -pi/2;
%             link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end
