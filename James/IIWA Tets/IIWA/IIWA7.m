classdef IIWA7 < RobotBaseClass
    properties(Access = public)              
         plyFileNameStem = 'IIWA';
    end
    
    methods
        function self = IIWA7(baseTr)
                        
            close all;
            clf;
            self.CreateModel();
            if nargin < 1            
                baseTr = eye(4);                
            end
            self.model.base = baseTr;
            
            self.PlotAndColourRobot();
            self.model.teach();  % Launch the teach GUI for manual control
        end

        function CreateModel(self)
            % Corrected DH Parameters for Kuka IIWA7
            link(1) = Link('d',0.1575,'a',0,'alpha',0,'qlim',deg2rad([-170 170]), 'offset',0);   % Joint 1
            link(2) = Link('d',0.2025,'a',0,'alpha',-pi/2,'qlim',deg2rad([-120 120]), 'offset',0);  % Joint 2
            link(3) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-170 170]), 'offset',0);  % Joint 3
            link(4) = Link('d',0.42,'a',0,'alpha',pi/2,'qlim',deg2rad([-120 120]), 'offset',0);  % Joint 4
            link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-170 170]), 'offset',0);  % Joint 5
            link(6) = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim',deg2rad([-120 120]), 'offset',0);  % Joint 6
            link(7) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]), 'offset',0);  % Joint 7
            link(8) = Link('d',0.1135,'a',0,'alpha',0,'qlim',deg2rad([-175 175]), 'offset',0);  % End-effector

            % Define the model
            self.model = SerialLink(link, 'name', 'Kuka IIWA7');
            self.model.plotopt = {'noaxes', 'noshadow', 'noarrow', 'noshading', 'nowrist', 'nojaxes'};
        end
    end
end
