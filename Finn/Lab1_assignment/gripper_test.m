classdef gripper_test < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'LinearUR3e';
        gripper;  % Add gripper property
    end
    
    methods
        %% Define robot function 
        function self = LinearUR3e(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);                
            end
            self.model.base = self.model.base * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();
            self.AttachGripper();  % Attach the gripper after plotting the robot
        end

        %% Create the robot model
        function CreateModel(self)
            % Create the UR5 model mounted on a linear rail
            link(1) = Link([pi 0 0 pi/2 1]); % PRISMATIC Link
            link(2) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            % Incorporate joint limits
            link(1).qlim = [-0.8 -0.01];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-170 170]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;
        
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.plyFileNameStem);
        end

        %% Attach the gripper to the robot
        function AttachGripper(self)
            % Define the gripper's transformation relative to the last link
            gripperTransform = transl(0, 0, 0.1);  % Adjust translation as needed
            
            % Create a basic gripper geometry (this is a simple approximation)
            self.gripper = SerialLink([Link('d',0,'a',0,'alpha',0)], 'name', 'gripper');
            
            % Define transformation from the end-effector to the gripper
            self.gripper.base = self.model.fkine(self.model.q) * gripperTransform;
            
            % Plot the gripper attached to the end-effector
            self.gripper.plot(0);  % '0' since it's a single fixed link
        end
        
    end
end
