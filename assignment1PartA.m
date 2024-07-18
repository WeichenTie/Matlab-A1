% assignment1PartA.m
% MTRN4230 Assignment 1 24T2
% Name: Weichen Tie
% Zid:  z5308889
startup_rvc
clear; clc;

host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
% port = 30003;
% rtde = rtde(host, port);

disp("Enter the pickup position")
pickupJointConfiguration =  [0.00, -75.00, 90.00, -105.00, -90.00, 0.00]
clc;
disp("Move robot to dropoff position")
dropoffJointConfiguration= readConfiguration()

clc;
disp("Calculated pickup pose: ")
pickupPose = convertJointToPose(pickupJointConfiguration)
disp("Calculated dropoff pose: ")
dropoffPose = convertJointToPose(dropoffJointConfiguration)

disp("Set robot to remote control mode then click enter")
input('');

% RTDE says it's taking in [x,y,z,r,p,y] but 
% its actually taking in [x,y,z,(rotation vector)]
% 
% The below four lines converts students rpy pose, into one with a rotation
% vector 
Tp = rpy2tr(pickupPose(4:6));
pickupPose = [pickupPose(1:3), rotmat2vec3d(Tp(1:3, 1:3))]'

Td = rpy2tr(dropoffPose(4:6));
dropoffPose = [dropoffPose(1:3), rotmat2vec3d(Td(1:3, 1:3))]'

rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(pickupPose', 'pose')
rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')

rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(dropoffPose', 'pose')
rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')

% Function to convert user input to array

function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j3,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% You must implement the following function
function outputPose = convertJointToPose(jointConfiguration)
    % Replace this with your implementation
    
    % Start with identity matrix and then
    theta = jointConfiguration;
    ure5_a = [0, -0.425, -0.3922, 0, 0, 0] * 1000;
    ure5_d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996] * 1000;
    ure5_alpha = rad2deg([pi / 2, 0 , 0, pi/2, -pi/2, 0]);
    foor = deg2rad(jointConfiguration)
    mat = eye(4);
    for index = 1:length(jointConfiguration)
        i = index
        trans_z_n_prev = [[1, 0, 0, 0        ];
                          [0, 1, 0, 0        ];
                          [0, 0, 1, ure5_d(i)];
                          [0, 0, 0, 1        ]];

        rot_z_n_prev = [[cosd(theta(i)), -sind(theta(i)), 0, 0];
                        [sind(theta(i)),  cosd(theta(i)), 0, 0];
                        [0             ,  0             , 1, 0];
                        [0             ,  0             , 0, 1]];
        
        trans_z = [[1, 0, 0, ure5_a(i)];
                   [0, 1, 0, 0        ];
                   [0, 0, 1, 0        ];
                   [0, 0, 0, 1        ]];

        rot_z = [[1, 0                  ,  0                  , 0];
                 [0, cosd(ure5_alpha(i)), -sind(ure5_alpha(i)), 0];
                 [0, sind(ure5_alpha(i)),  cosd(ure5_alpha(i)), 0];
                 [0, 0                  ,  0                  , 1]];
        
        T_n = trans_z_n_prev * rot_z_n_prev * trans_z * rot_z
        mat = mat * T_n
    end
    outputPose = [mat(1:3,4)', tr2rpy(mat)]
    % You must not use RTDE at all in this implementation (it
    % must be done from first principles)
end


