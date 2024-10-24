function iiwa_movement(selected_spread, selected_brick, robotModel)
    % selected_spread: 'Butter' or 'Jam'
    % selected_brick: Index of the selected brick
    % robotModel: The IIWA robot model

    % Define matrix positions for Butter and Jam (specific to IIWA's workspace)
    butter_position1 = transl(0.3, 0.2, 0.5);  % Example position 1 for butter
    butter_position2 = transl(0.4, 0.2, 0.6);  % Example position 2 for butter
    jam_position1 = transl(0.5, -0.2, 0.5);    % Example position 1 for jam
    jam_position2 = transl(0.6, -0.2, 0.6);    % Example position 2 for jam
    
    % Define final brick position based on the brick selected
    brickMatrix = [
        -1.7, -0.5, 0.8;
        -1.7, -0.33, 0.8
    ];
    final_position = transl(brickMatrix(selected_brick, :));  % Final position of the selected brick

    % Move the IIWA through Butter or Jam positions first
    if strcmp(selected_spread, 'Butter')
        move_to_position(robotModel, butter_position1);
        move_to_position(robotModel, butter_position2);
    elseif strcmp(selected_spread, 'Jam')
        move_to_position(robotModel, jam_position1);
        move_to_position(robotModel, jam_position2);
    end
    
    % Finally, move the IIWA to the brick position
    move_to_position(robotModel, final_position);
    
    disp('IIWA Robot has completed the motion.');
end

function move_to_position(robotModel, target_position)
    % Use inverse kinematics to compute joint angles for the target position
    qSol = robotModel.model.ikcon(target_position);
    
    % Animate the robot to the new joint configuration
    robotModel.model.animate(qSol);
    
    pause(1);  % Add a small delay for each movement step (adjust as needed)
end
