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




MPInitialize(C_ini);
ParaInitialize(C_ini, C_goal, Obs);

iter = 0;
goalBias = 0.10;

params.robot = C_ini;
if HasRobotReachedGoal()
    mp.vidAtGoal = 1;
end

while mp.vidAtGoal <= 0 && iter < params.maxiteration

    % Use the actual goal occasionally; otherwise reaching it by pure
    % random sampling in six dimensions is extremely unlikely.
    if rand < goalBias
        sto = C_goal;
    else
        sto = SampleState();
    end

    distances = vecnorm(mp.nodes - sto, 2, 2);
    [~, vidNear] = min(distances);

    MPExtendTree(vidNear, sto);
    iter = iter + 1;

    if mod(iter, 200) == 0
        fprintf('RRT iteration %d: nodes=%d, goalReached=%d\n', iter, size(mp.nodes, 1), mp.vidAtGoal);
    end
end

if mp.vidAtGoal >= 1
    JointTrajectory = MPGetPath();
    JointTrajectory_smooth = SmoothPath(JointTrajectory);
    Draw(JointTrajectory_smooth);
    fprintf('RRT succeeded after %d iterations with %d nodes.\n', iter, size(mp.nodes, 1));
else
    fprintf('RRT failed after %d iterations. Nodes grown: %d\n', iter, size(mp.nodes, 1));
end
end