function [JointTrajectory, JointTrajectory_smooth] = MPExtendRRT(C_ini, C_goal, Obs)
% C_ini is the initial configuration of UR5 (dimensino: 1*6, unit: radian)
% C_goal is the goal configuration of UR5 (dimensino: 1*6, unit: radian)
% Obs represents all the capsule obstacles in the workspace (dimension: n*7)
% format of each row of Obs: [P_ini,  P_end,  r] where P_ini is the x-, y-
% and z- coordinates of the initial point of the interal line segment of
% the capsule (diemsion: 1*3 unit: meter), P_end is the x-, y-
% and z- coordinates of the end point of the interal line segment of
% the capsule (diemsion: 1*3 unit: meter), r is the radius of the capsule
% (scalar, unit: meter)

% please define all these three inputs before runing this main function

global mp;
global params;

JointTrajectory = []; 
JointTrajectory_smooth = [];
% JointTrajectory is the original solution path and JointTrajectory_smooth 
% is the path after post-processing the path JointTrajectory

% JointTrajectory_smooth is the joint trajectories of all the six joints of UR5, 
% which you should transform them into the format of UR script and deploy them on the real robot.

% JointTrajectory_smooth is a matrix whose dimension is n * 6 (unit: radian)

% 1. DEFINE OBSTACLES WITH A CLEAR CORRIDOR
    Obs_workbench = [0, -1.0, -0.10, 0, 1.0, -0.10, 0.10];
    
    theta = 22.5 * (pi / 180);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % Spread the pillars slightly wider apart so there's an easy path between them
    physical_pillar1_xy = [0.4;  0.5]; 
    physical_pillar2_xy = [0.4; -0.5];
    
    robot_pillar1_xy = R * physical_pillar1_xy;
    robot_pillar2_xy = R * physical_pillar2_xy;
    
    pillar_radius = 0.04; % Made them slightly thinner (4cm) to give more breathing room
    Obs_pillar1 = [robot_pillar1_xy(1), robot_pillar1_xy(2), 0, robot_pillar1_xy(1), robot_pillar1_xy(2), 1.0, pillar_radius];
    Obs_pillar2 = [robot_pillar2_xy(1), robot_pillar2_xy(2), 0, robot_pillar2_xy(1), robot_pillar2_xy(2), 1.0, pillar_radius];

    % Combine into Obs
    Obs = [Obs_workbench; Obs_pillar1; Obs_pillar2];
MPInitialize(C_ini);
ParaInitialize(C_ini, C_goal, Obs);

iter = 0;

while mp.vidAtGoal <= 0

% Implement the extension of the RRT algorithm inside while loop here ...
% 1. Generate a random target state
if rand() < 0.1
            q_rand = params.goal; 
        else
    q_rand = SampleState();
end
    % 2. Find the nearest node in our existing tree (mp.nodes)
    [num_nodes, ~] = size(mp.nodes);
    min_dist = inf;
    vid_nearest = 1;
    
    for i = 1:num_nodes
        % Calculate Euclidean distance
        dist = norm(mp.nodes(i, :) - q_rand);
        if dist < min_dist
            min_dist = dist;
            vid_nearest = i; % Save the index of the closest node
        end
    end
    
    % 3. Try to grow the tree from our closest node toward the random guess
    MPExtendTree(vid_nearest, q_rand);
    
    % 4. Prevent an infinite loop by using the max iterations limit
    iter = iter + 1;
    if iter >= params.maxiteration
        disp('Failed to find a path within the maximum number of iterations.');
        break;
    end

end


if mp.vidAtGoal >= 1
    JointTrajectory  = MPGetPath();
    JointTrajectory_smooth = SmoothPath(JointTrajectory);


Draw(JointTrajectory_smooth);
else
    
    disp('The algorithm finished, but no safe path was found. Please try again or move the obstacles.');

end
