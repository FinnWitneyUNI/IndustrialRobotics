classdef RMRCController < handle
    properties (Access = private)
        robot           
        deltaT         
        manipEpsilon   
        lambda         
        weightMatrix   
    end
    
    properties (Access = public)
        qMatrix        
        cartesianError 
    end
    
    methods
        function self = RMRCController(robot, deltaT)
            if nargin < 2
                deltaT = 0.02;  
            end
            
            self.robot = robot;
            self.deltaT = deltaT;
            self.manipEpsilon = 0.02;    % Reduced back to original
            self.lambda = 0.01;          % Reduced back to original
            
            % Balanced weight matrix
            self.weightMatrix = diag([1 1 1 0.1 0.1 0.1]);
        end
        
        function [qMatrix, cartesianError] = computeTrajectory(self, startTr, endTr, totalTime)
            steps = round(totalTime/self.deltaT);
            numJoints = self.robot.n;
            
            % Initialize matrices
            self.qMatrix = zeros(steps, numJoints);
            self.cartesianError = zeros(6, steps);
            
            % Get initial joint state
            q0 = self.robot.getpos();
            self.qMatrix(1,:) = q0;
            
            % Extract start and end positions
            startPos = transl(startTr);
            endPos = transl(endTr);
            
            % Main RMRC loop
            for i = 1:steps-1
                % Current end-effector transform
                currentTr = self.robot.fkine(self.qMatrix(i,:));
                currentPos = transl(currentTr);
                
                % Linear interpolation
                s = i/steps;
                
                % Calculate target position
                targetPos = (1-s)*startPos(:) + s*endPos(:);
                
                % Position error
                dp = targetPos - currentPos(:);
                
                % Orientation error
                currentR = t2r(currentTr);
                targetR = t2r(endTr);
                dR = targetR * currentR';
                [theta, n] = tr2angvec(dR);
                dw = theta * n(:);
                
                % Ensure vectors are 3x1
                dp = reshape(dp, 3, 1);
                dw = reshape(dw, 3, 1);
                
                % Form velocity vector
                xdot = zeros(6,1);
                xdot(1:3) = dp/self.deltaT;
                xdot(4:6) = dw/self.deltaT;
                
                % Get Jacobian
                J = self.robot.jacob0(self.qMatrix(i,:));
                
                % Basic singularity handling
                m = sqrt(det(J*J'));
                lambda = self.lambda;
                if m < self.manipEpsilon
                    lambda = self.lambda * (1 - m/self.manipEpsilon)^2;
                end
                
                % Compute joint velocities
                Jinv = inv(J'*J + lambda*eye(numJoints))*J';
                qdot = Jinv * (self.weightMatrix * xdot);
                
                % Update joint positions
                self.qMatrix(i+1,:) = self.qMatrix(i,:) + qdot'*self.deltaT;
                
                % Apply joint limits
                for j = 1:numJoints
                    if self.qMatrix(i+1,j) < self.robot.qlim(j,1)
                        self.qMatrix(i+1,j) = self.robot.qlim(j,1);
                    elseif self.qMatrix(i+1,j) > self.robot.qlim(j,2)
                        self.qMatrix(i+1,j) = self.robot.qlim(j,2);
                    end
                end
                
                % Store error
                self.cartesianError(:,i) = xdot;
            end
            
            qMatrix = self.qMatrix;
            cartesianError = self.cartesianError;
        end
        
        function animate(self, pauseTime)
            if nargin < 2
                pauseTime = 0.01;
            end
            
            for i = 1:size(self.qMatrix,1)
                self.robot.animate(self.qMatrix(i,:));
                drawnow();
                pause(pauseTime);
            end
        end
        
        function plotError(self)
            figure('Name', 'RMRC Errors');
            
            subplot(2,1,1);
            plot(self.cartesianError(1:3,:)');
            title('Position Error');
            xlabel('Step');
            ylabel('Error (m)');
            legend('X', 'Y', 'Z');
            grid on;
            
            subplot(2,1,2);
            plot(self.cartesianError(4:6,:)');
            title('Orientation Error');
            xlabel('Step');
            ylabel('Error (rad)');
            legend('Roll', 'Pitch', 'Yaw');
            grid on;
        end
    end
end