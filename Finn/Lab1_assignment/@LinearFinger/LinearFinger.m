classdef LinearFinger < RobotBaseClass

    properties(Access = public)              
        plyFileNameStem = 'LinearFinger';
    end
    
    methods
        % Define robot function 
        function self = LinearFinger(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = transl(0,0,0);                
            end
            self.model.base =  self.model.base.T * baseTr;

            self.PlotAndColourRobot();         
        end

        % Create the robot model
        function CreateModel(self)   
            % Adjust the link lengths as needed
            link(1) = Link('d',0,'a',0.04,'alpha',0,'qlim',deg2rad([-30 30]),'offset',deg2rad(0));   
            link(2) = Link('d',0,'a',0.04,'alpha',0,'qlim',deg2rad([0 40]),'offset',deg2rad(0));
            
            self.model = SerialLink(link,'name',self.plyFileNameStem);
            self.model.plotopt = {'noshadow','noarrow','noshading','nowrist','nojaxes'};
        end
    end
end
