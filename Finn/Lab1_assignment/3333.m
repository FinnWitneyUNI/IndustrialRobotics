            %Gripper Attempt
            a1 = 0.1;
            a2 = 0.1;

            p2 = SerialLink([
                 Revolute('d', 0, 'a', a1, 'alpha', 0, 'standard')
                 Revolute('d', 0, 'a', a2, 'alpha', 0, 'standard')
                 ], ...
                 'name', 'planar 2 link');
            qz = [0 0];

            qMatrix = jtraj(r.model.qlim(:,1),r.model.qlim(:,2),100);

            for i = 100
                r.model.animate(qMatrix(1,:));
                p2.base = r.model.fkine(qMatrix(1,:)).T;
                p2.plot(qz);
                drawnow();
            end