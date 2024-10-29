classdef RMRCController
    properties
        robot;  % Robot model
        timeStep;  % Time step for the RMRC control
        lambda = 0.1;  % Damping factor for DLS (adjust based on your needs)
    end

    methods
        function obj = RMRCController(robot, timeStep)
            obj.robot = robot;
            obj.timeStep = timeStep;
        end

        function [qMatrix, success] = computeTrajectory(obj, startTr, endTr, duration)
            % Initialize
            numSteps = round(duration / obj.timeStep);
            s = lspb(0, 1, numSteps);  % S-curve for interpolation
            qMatrix = zeros(numSteps, length(obj.robot.getpos));
            q0 = obj.robot.ikcon(startTr);  % Initial joint configuration
            qMatrix(1, :) = q0;

            % Extract roll, pitch, and yaw for start and end transforms
            startRPY = tr2rpy(startTr, 'deg');
            endRPY = tr2rpy(endTr, 'deg');

            % Trajectory generation
            for i = 2:numSteps
                % Interpolate the Cartesian position and orientation
                T = startTr * transl(s(i) * (endTr(1:3, 4) - startTr(1:3, 4))) * ...
                    trotx(s(i) * (endRPY(1) - startRPY(1))) * ...
                    troty(s(i) * (endRPY(2) - startRPY(2))) * ...
                    trotz(s(i) * (endRPY(3) - startRPY(3)));

                % Retrieve and convert the current end-effector transformation matrix
                currentTransform = double(obj.robot.fkine(qMatrix(i-1, :)));

                % Compute desired end-effector velocity
                deltaX = (T(1:3, 4) - currentTransform(1:3, 4)) / obj.timeStep;

                % Ensure deltaTheta is a 3x1 vector
                rotationError = tr2rpy(T * inv(currentTransform), 'deg');
                deltaTheta = rotationError(:) / obj.timeStep;
                
                % Concatenate deltaX and deltaTheta
                xdot = [deltaX; deltaTheta];

                % Calculate the Jacobian and apply Damped Least-Squares (DLS)
                J = obj.robot.jacob0(qMatrix(i-1, :));
                J_pseudo = (J' * J + obj.lambda^2 * eye(size(J, 2))) \ J';  % DLS inverse
                dq = J_pseudo * xdot;  % Joint velocities

                % Update joint positions
                qMatrix(i, :) = qMatrix(i-1, :) + dq' * obj.timeStep;
            end

            success = true;
        end
    end
end
