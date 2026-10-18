%% Get Poses of Links Body1 to Body5 (FME 500)
% This script computes forward kinematics for each link

% --- Step 1: Load the robot ---
robot = importrobot('Mulag_FME500_kinematic.slx');
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% --- Step 2: Define a joint configuration ---
% Use home configuration or specify your own
q = robot.homeConfiguration;

% Alternatively, use your IK result:
% q = [1.5708, -0.4900, 0.0480, 0.3847, 1.5708];

% --- Step 3: Loop through each body and get pose ---
disp('Link Poses relative to base:');

for i = 1:5
    bodyName = ['Body', num2str(i)];

    % Get 4×4 homogeneous transform relative to base
    T = getTransform(robot, q, bodyName);

    % Extract position and orientation
    pos = tform2trvec(T);               % [x y z]
    rpy = rad2deg(tform2eul(T, 'ZYX')); % [yaw pitch roll] in degrees

    fprintf('\n%s:\n', bodyName);
    fprintf('  Position [x y z] = [%.3f, %.3f, %.3f] m\n', pos);
    fprintf('  Orientation [yaw pitch roll] = [%.2f°, %.2f°, %.2f°]\n', ...
        rpy(1), rpy(2), rpy(3));
end
