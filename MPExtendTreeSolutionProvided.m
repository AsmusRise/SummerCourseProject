function [] = MPExtendTree(vid, sto)
% Extend the tree from the state with index vid toward the state sto
% At each time, make a small step with magnitude params.distOneStep
% Add each intermediate valid state to the tree data structure.
% Stop as soon as an intermediate invalid state is encountered
%   - You can check state validity by first setting the robot position and
%   - then calling the function IsStateValid
% Also stop if an intermediate state reaches the goal
%   - You can check if robot has reached the goal by first setting the
%   - robot position and then calling the function HasRobotReachedGoal

global mp;
global params;

mp.sto = sto;
mp.vidNear = [mp.xpts(vid) mp.ypts(vid)];

% You can make use of the global variables params and mp to access the
% necessary information

% Add your code here

dstep     = params.distOneStep;
x         = sto(1) - mp.xpts(vid);
y         = sto(2) - mp.ypts(vid);
d         = norm([x, y]);
ux        = dstep * x / d;
uy        = dstep * y / d;
nrSteps = 10; %ceil(d / dstep);

for k = 1 : 1 : nrSteps
    params.robot(1) = mp.xpts(vid) + ux;
    params.robot(2) = mp.ypts(vid) + uy;
    if IsValidState() == 0
        return;
    end
    n                     = length(mp.xpts);
    mp.nchildren(vid)     = mp.nchildren(vid) + 1;
    mp.xpts(n + 1)        = params.robot(1);
    mp.ypts(n + 1)        = params.robot(2);
    mp.parents(n + 1)     = vid;
    mp.nchildren(n + 1)   = 0;
    
    if HasRobotReachedGoal() == 1
        mp.vidAtGoal = n + 1;
        return;
    end
    vid = n + 1;
end

end

