function [] = MPExtendTree(vid, sto)

global mp;
global params;

mp.sto = sto;
mp.vidNear = mp.nodes(vid,:);

currentVid = vid;
currentState = mp.nodes(currentVid,:);

while norm(sto - currentState) > 1e-10
    direction = sto - currentState;
    stepLength = min(params.distOneStep, norm(direction));
    candidate = currentState + stepLength * direction / norm(direction);

    params.robot = candidate;

    if IsValidState() == 0
        return
    end

    mp.nodes = [mp.nodes; candidate];
    mp.parents = [mp.parents; currentVid];
    mp.nchildren(currentVid) = mp.nchildren(currentVid) + 1;
    mp.nchildren = [mp.nchildren; 0];

    currentVid = size(mp.nodes, 1);
    currentState = candidate;

    if HasRobotReachedGoal()
        mp.vidAtGoal = currentVid;
        return
    end
end


if mp.vidAtGoal >= 1
    JointTrajectory = MPGetPath();
    JointTrajectory_smooth = SmoothPath(JointTrajectory);
    Draw(JointTrajectory_smooth);
end
end
