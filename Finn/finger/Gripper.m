classdef Gripper < handle
    properties
        finger1  % Left gripper finger
        finger2  % Right gripper finger
        gripper_finger_spacing = 0.1;  % Distance between the fingers
    end

    methods
        % Constructor to initialize the gripper with both fingers
        function self = Gripper()
            % Initialize left and right fingers (as LinearFinger models)
            self.finger1 = LinearFinger();  % Left finger
            self.finger2 = LinearFinger2();  % Right finger
        end

        % Method to attach the gripper to the robot's end-effector
        function attachToEndEffector(self, endEffectorTransform)
            % Position the left gripper finger (-x direction relative to the end-effector)
            baseTr1 = endEffectorTransform * transl(-self.gripper_finger_spacing / 2, 0, 0) * trotx(-pi/2) * trotz(-pi/2);
            self.finger1.model.base = baseTr1;
            self.finger1.model.animate([0, 0]);
            drawnow();

            % Position the right gripper finger (+x direction relative to the end-effector)
            baseTr2 = endEffectorTransform * transl(self.gripper_finger_spacing / 2, 0, 0) * trotx(pi/2) * trotz(pi/2);
            self.finger2.model.base = baseTr2;
            self.finger2.model.animate([0, 0]);
            
            drawnow();  % Render both fingers together
        end

        % Method to animate both fingers together
        function animateGripper(self)
            self.finger1.model.animate([0, 0]);
            self.finger2.model.animate([0, 0]);
            drawnow();  % Update the rendering
        end
    end
end
