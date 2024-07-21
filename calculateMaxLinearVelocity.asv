% calculateMaxLinearVelocity.m
% MTRN4230 Assignment 1 24T2
% Name: Weichen Tie
% Zid:  z5308889

%% Function you must complete
% You must implement the following function

function maxLinearVelocity = calculateMaxLinearVelocity(jointPositions,jointVelocities)
    % Defining the link array in accordance with the DH table
    L(1) = Link([0, 0.1625, 0,  pi/2]); % Link 1
    L(2) = Link([0, 0, -0.425,  0]); % Link 2
    L(3) = Link([0,  0, -0.3922, 0]); % Link 3
    L(4) = Link([0, 0.1333, 0,  pi/2]); % Link 4
    L(5) = Link([0, 0.0997, 0,  -pi/2]); % Link 5
    L(6) = Link([0, 0.0996, 0,  0]); % Link 6
    % Creating the robot
    robot = SerialLink(L, 'name', 'Articulated');
    maxLinearVelocity = 0;
    % Iterate through the jointPositions and jointVelocities arrays.
    for i = 1:length(jointPositions)
        % Calculate the Jacobian matrix
        jacobian = jacob0(robot, jointPositions(i,:));
        % Calculate tool/end effector linear velocity and angular velocities 
        toolVelocity = jacobian * jointVelocities(i,:)';
        % Tool/end effector linear velocities encoded in first 3 rows of
        % vector
        linearVelocities = toolVelocity(1:3);
        % Calculate the magnitude of this velocity
        dotProduct = sum(linearVelocities .* linearVelocities);
        magnitude =sqrt(dotProduct);
        % Update the maxLinearVelocity value
        maxLinearVelocity = max(maxLinearVelocity, magnitude);
    end
end