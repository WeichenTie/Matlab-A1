% calculateMaxLinearVelocity.m
% MTRN4230 Assignment 1 24T2
% Name: 
% Zid:  zXXXXXXX

%% Function you must complete
% You must implement the following function
function maxLinearVelocity = calculateMaxLinearVelocity(jointPositions,jointVelocities)

    d = []


    
    L(1) = Link('revolute', 'd', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', 0); % Link 1. Offset of theta1
    L(2) = Link('revolute', 'd', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', 0); % Link 2. Offset of theta2
    L(3) = Link('revolute', 'd', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', 0); % Link 3. Offset of theta3
    L(4) = Link('revolute', 'd', d(4), 'a', a(3), 'alpha', alpha(4), 'offset', 0); % Link 4. Offset of theta1
    L(5) = Link('revolute', 'd', d(5), 'a', a(4), 'alpha', alpha(5), 'offset', 0); % Link 5. Offset of theta2
    L(6) = Link('revolute', 'd', d(6), 'a', a(5), 'alpha', alpha(6), 'offset', 0); % Link 6. Offset of theta3



    % Write your implementation here
    maxLinearVelocity = 0;
    

    disp(jointPositions)
    disp(jointVelocities)
end