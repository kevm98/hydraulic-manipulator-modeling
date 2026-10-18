%% Inverse Kinematics Script for FME 500 Manipulator

% --- Load robot model ---
robot = importrobot('Mulag_FME500_kinematic.slx');  % adjust if needed
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% --- Set joint limits (in radians) ---
robot.Bodies{1}.Joint.PositionLimits = deg2rad([-120, 120]);  % Drehzapfen
robot.Bodies{2}.Joint.PositionLimits = deg2rad([-40, 90]);    % Ausleger I
robot.Bodies{3}.Joint.PositionLimits = deg2rad([-60, 110]);   % Ausleger II
robot.Bodies{4}.Joint.PositionLimits = deg2rad([-100, 100]);  % Ausleger III
robot.Bodies{5}.Joint.PositionLimits = deg2rad([-90, 90]);    % Messerkopf

% --- Define the IK solver ---
ik = inverseKinematics('RigidBodyTree', robot);
endEffectorName = 'Body5';  % adjust to match your actual frame name
weights = [1 1 1 0.1 0.1 0.1];   % prioritize position over orientation
initialGuess = robot.homeConfiguration;

% --- Step 4: Define desired end-effector pose ---
desiredPos = [4.0, 0.0, 1.5];  % [x y z] in meters
desiredEulerZYX = deg2rad([0, 0, 180]);  % [yaw pitch roll] in degrees → radians
desiredTform = trvec2tform(desiredPos) * eul2tform(desiredEulerZYX, 'ZYX');

% --- Solve IK ---
[configSol, solInfo] = ik(endEffectorName, desiredTform, weights, initialGuess);

% --- Display results ---
disp('Inverse Kinematics Solution:');
disp('Joint Angles (radians):');
disp(configSol);

disp('Solver Info:');
disp(solInfo);

% --- Visualize ---
figure;
show(robot, configSol, 'Frames', 'on', 'Visuals', 'on');
title('Inverse Kinematics Result');
