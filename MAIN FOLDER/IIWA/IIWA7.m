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
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();         
        end


function CreateModel(self)   
    % Define the DH parameters for the Kuka IIWA7
    link(1) = Link('d',0.1575,'a',0,'alpha',0,'qlim',deg2rad([-170 170]), 'offset',0);   % Joint 1
    link(2) = Link('d',0.2025,'a',0,'alpha',-pi/2,'qlim', deg2rad([-120 120]), 'offset',0);  % Joint 2
    link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim', deg2rad([-170 170]), 'offset', 0);  % Joint 3
    link(4) = Link('d',0.42,'a',0,'alpha',pi/2,'qlim',deg2rad([-120 120]),'offset', 0);  % Joint 4
    link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-170 170]), 'offset',0);  % Joint 5
    link(6) = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim',deg2rad([-120 120]), 'offset', 0);  % Joint 6
    link(7) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]), 'offset', 0);  % Joint 7
    link(8) = Link('d',0.1135,'a',0,'alpha',0,'qlim',deg2rad([-175 175]), 'offset', 0);  % End Effector Link

    
    
    
    % Incorporate joint limits
    link(1).qlim = deg2rad([-170 170]);
    link(2).qlim = deg2rad([-120 120]);
    link(3).qlim = deg2rad([-170 170]);
    link(4).qlim = deg2rad([-120 120]);
    link(5).qlim = deg2rad([-170 170]);
    link(6).qlim = deg2rad([-120 120]);
    link(7).qlim = deg2rad([-175 175]);
    
    % Define model name as Kuka IIWA7
    self.model = SerialLink(link, 'name', 'Kuka IIWA7');
end
     
    end
end
