% calculateMaxLinearVelocity.m
% MTRN4230 Assignment 1 24T2
% Name: Weichen Tie
% Zid:  z5308889

%% Function you must complete
% You must implement the following function

function maxLinearVelocity = calculateMaxLinearVelocity(jointPositions,jointVelocities)
    
    L(1) = Link([0, 0.1625, 0,  pi/2]); % Link 1
    L(2) = Link([0, 0, -0.425,  0]); % Link 2
    L(3) = Link([0,  0, -0.3922, 0]); % Link 3
    L(4) = Link([0, 0.1333, 0,  pi/2]); % Link 4
    L(5) = Link([0, 0.0997, 0,  -pi/2]); % Link 5
    L(6) = Link([0, 0.0996, 0,  0]); % Link 6

    robot = SerialLink(L, 'name', 'Articulated');
    maxLinearVelocity = 0;
    for i = 1:length(jointPositions)
        jacobian = jacob0(robot, jointPositions(i,:));
        toolVelocity = jacobian * jointVelocities(i,:)';
        linearVelocities = toolVelocity(1:3);
        dotProduct = sum(linearVelocities .* linearVelocities);
        magnitude =sqrt(dotProduct);
        maxLinearVelocity = max(maxLinearVelocity, magnitude);
    end
end

% calculateMaxLinearVelocity1(jointPositions, jointVelocities)