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




planningTic = tic;
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
    planningSeconds = toc(planningTic);
    if isfield(params, 'showSimulation') && params.showSimulation == 1
        Draw(JointTrajectory_smooth);
    end
    logRRTRun(C_ini, C_goal, iter, mp, JointTrajectory, JointTrajectory_smooth, true, planningSeconds);
    fprintf('RRT succeeded after %d iterations with %d nodes.\n', iter, size(mp.nodes, 1));
else
    planningSeconds = toc(planningTic);
    logRRTRun(C_ini, C_goal, iter, mp, JointTrajectory, JointTrajectory_smooth, false, planningSeconds);
    fprintf('RRT failed after %d iterations. Nodes grown: %d\n', iter, size(mp.nodes, 1));
end
end

function logRRTRun(C_ini, C_goal, iter, mp, JointTrajectory, JointTrajectory_smooth, successFlag, planningSeconds)
global params;

if ~isfield(params, 'logFile') || isempty(params.logFile)
    return;
end

logFid = fopen(params.logFile, 'a');
if logFid == -1
    warning('Could not append to log file: %s', params.logFile);
    return;
end

if successFlag
    moveJCount = size(JointTrajectory_smooth, 1);
else
    moveJCount = 0;
end

fprintf(logFid, 'Run finished: %s\n', datestr(now));
fprintf(logFid, 'Status: %s\n', ternary(successFlag, 'success', 'failure'));
fprintf(logFid, 'Start: [%.6f %.6f %.6f %.6f %.6f %.6f]\n', C_ini);
fprintf(logFid, 'Goal:  [%.6f %.6f %.6f %.6f %.6f %.6f]\n', C_goal);
fprintf(logFid, 'Iterations: %d\n', iter);
fprintf(logFid, 'Planning seconds: %.6f\n', planningSeconds);
fprintf(logFid, 'Nodes: %d\n', size(mp.nodes, 1));
fprintf(logFid, 'Raw path length: %d\n', size(JointTrajectory, 1));
fprintf(logFid, 'Smoothed path length: %d\n', size(JointTrajectory_smooth, 1));
fprintf(logFid, 'moveJ count: %d\n', moveJCount);
fprintf(logFid, 'Goal reached: %d\n', mp.vidAtGoal >= 1);
fprintf(logFid, '---\n');
fclose(logFid);
end

function out = ternary(condition, trueValue, falseValue)
if condition
    out = trueValue;
else
    out = falseValue;
end
end