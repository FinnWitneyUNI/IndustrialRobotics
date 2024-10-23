function [h_] = PlotVolume(self, createNew, color)
                
            %PlotVolume(DobotMagician,1,'red');  THIS IS HOW TO RUN THE CODE
                       
            %base = transl(self.model.base); 
            stepRads = deg2rad(10);
        
            qlim =[deg2rad(-135) deg2rad(135) ; deg2rad(5) deg2rad(80); deg2rad(-5) deg2rad(85); -pi pi; deg2rad(-85) deg2rad(85)];
            
            %qlim = self.qlim;
            if createNew == 1 % only if pointCloud has not been create yet
                pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
                pointCloud = zeros(pointCloudeSize,3);
                disp(num2str(size(pointCloud)));
                counter = 1;
                tic
                for q1 = qlim(1,1):stepRads:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            for q4 = qlim(4,1):stepRads:qlim(4,2)
                                for q5 = qlim(5,1):stepRads:qlim(5,2)
                                                                            
                                        
                                        q = [q1,q2,q3,q4,q5];
                                        tr = self.model.fkineUTS(q);                       
                                        pointCloud(counter,:) = tr(1:3,4)';
                                        counter = counter + 1;               
                                        
                                        if mod(counter/pointCloudeSize * 100,1) == 0
                                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                        end
                                    
                                end
                            end
                        end
                    end
                end
               end
           
            hold on;
            if color == "red"
                color = [255 0 0]/255;
            else if color == "blue"
                color = [0 0 255]/255;
                end
            end
            figure(1);
            
            h_ = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'o','Color',color);
            drawnow();
            
            hold on;
            %max x radius  
            a = (max(pointCloud(:,1)) - min(pointCloud(:,1)))/2
            %max y radius
            b = (max(pointCloud(:,2)) - min(pointCloud(:,2)))/2
            %max z radius 
            c = (max(pointCloud(:,3)) - min(pointCloud(:,3)))/2
            %Assume it is a sphere, calculate it volume as a sphere
            volume = (4/3)*pi*a*b*c;
            %display the volume value in m^3
            disp(strcat("Approximate volume (m^3) = ",num2str(volume)));
end
