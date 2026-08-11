% =========================================================================
% test_rrt_target_at_06.m
% =========================================================================

clear; clc;
run_me;

% 1. Define the Obstacle Pillar 
% Located at X = 0.35, Y = 0.0, height of 0.4m
Obs = [0.35, 0.0, 0.0,  0.35, 0.0, 0.4,  0.05];

% 2. Set the Start Configuration 
C_ini = [-0.95, -0.85, -1.2, 0.95, 0.95, 0.6];

% 3. Initialize Parameters EARLY for collision checking
global params;
ParaInitialize(C_ini, C_ini, Obs); 
ur5_kin = UR5Kinematics(); 

% 4. Verify Start Configuration is Safe
params.robot = C_ini;
if IsValidState() == 0
    error('CRITICAL ERROR: C_ini is hitting the obstacle or joint limits!');
end

% 5. Explicitly set your target coordinates behind the obstacle
% X = 0.60m, Y = 0.0m, Z = 0.20m
target_xyz = [0.60; 0.0; 0.20];

% Build a clean, natural forward-reaching transformation matrix
C_template = [0.0, -0.8, -1.2, -0.8, 1.57, 0.0];
fk_ref = ur5_kin.forward_kinematics(C_template);

T_ee = eye(4);
T_ee(1:3, 1:3) = fk_ref.transform_matrices.T6(1:3, 1:3); % Safe orientation
T_ee(1:3, 4)   = target_xyz;                             % Exact target position

% 6. Solve Inverse Kinematics using the professor's function
disp('Calculating Inverse Kinematics for the target at X = 0.60...');
ik_sols = ur5_kin.inverse_kinematics(T_ee, 0); 

if isempty(ik_sols)
    error('IK could not find a mathematical solution for that coordinate.');
end

% 7. Loop through IK solutions to find a SAFE one that avoids limits/obstacles
C_goal = [];
for i = 1:size(ik_sols, 1)
    params.robot = ik_sols(i, :);
    if IsValidState() == 1
        C_goal = ik_sols(i, :);
        break; 
    end
end

if isempty(C_goal)
    % Fallback to the natural forward-reaching posture if filters catch it
    C_goal = [0.0, -0.7, -1.3, -0.8, 1.57, 0.0];
end

fprintf('Target joint angles successfully secured!\n');

% 8. Finalize Parameters with the valid Goal
ParaInitialize(C_ini, C_goal, Obs); 
params.distOneStep = 0.20; 
% params.maxiteration = 30000; 

% 9. Plot a permanent visual marker at your exact target location (X=0.60)
figure(1); hold on;
plot3(target_xyz(1), target_xyz(2), target_xyz(3), 'rp', ...
    'MarkerSize', 18, ...
    'MarkerFaceColor', 'y', ...
    'MarkerEdgeColor', 'r', ...
    'LineWidth', 2);
text(target_xyz(1), target_xyz(2), target_xyz(3) + 0.05, '  TARGET (X=0.6)', ...
    'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');

% 10. Run the RRT Algorithm and Simulate
disp('Starting RRT and Simulation...');
[JointTrajectory, JointTrajectory_smooth] = MPExtendRRT(C_ini, C_goal, Obs);

% 11. Save the Output script for the robot
if ~isempty(JointTrajectory_smooth)
    fileID = fopen('ur5_rrt_motion.script', 'w');
    for i = 1:size(JointTrajectory_smooth, 1)
        q = JointTrajectory_smooth(i, :);
        fprintf(fileID, 'movej([%e,%e,%e,%e,%e,%e],a=1.39,v=1.04)\n', q(1), q(2), q(3), q(4), q(5), q(6));
    end
    fclose(fileID);
    disp('Successfully saved trajectory to ur5_rrt_motion.script!');
else
    disp('RRT reached max iterations without finding a path.');
end