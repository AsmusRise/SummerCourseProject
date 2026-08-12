%% UR5 potential-field motion-planning runner
% This file is a clean, repo-specific version for the current UR5 setup.
% It builds the same collision obstacles as run_rrt_project.m,
% calls PotentialField.m, and exports the final trajectory as a URScript.

clear
clc
close all

projectFolder = fileparts(mfilename('fullpath'));
addpath(genpath(projectFolder));

%% 1. Start and goal joint configurations (radians)
C_ini  = [-deg2rad(90), deg2rad(-45), deg2rad(-135), deg2rad(-90), deg2rad(90), deg2rad(-25)];
C_goal = [-deg2rad(61.98), deg2rad(-125.8), deg2rad(-78.96), deg2rad(-66.35), deg2rad(87.74), deg2rad(1.71)];

%% 2. Build the same obstacle set as the RRT project
mm2m = 1e-3;

A = [242; 400; -30] * mm2m;
B = [-493; 90; -34] * mm2m;
C = [685; -707; -28] * mm2m;
D = [-38; -1014; -26] * mm2m;

p1 = [568; -394; -30] * mm2m;
p2 = [605; -481; -23] * mm2m;
p3 = [-175; -690; -28] * mm2m;
p4 = [-144; -783; -30] * mm2m;

b1 = [144; -482; -15] * mm2m;

nPoints = 10;
ptsAB = ((1:nPoints)'/(nPoints+1)) .* (B - A)' + A';
ptsCD = ((1:nPoints)'/(nPoints+1)) .* (D - C)' + C';

edgeRadius = 0.02;
pillarRadius = 0.03;
bottleradius = 0.042;

Obs = [
    A(1), A(2), A(3), B(1), B(2), B(3), edgeRadius
    B(1), B(2), B(3), D(1), D(2), D(3), edgeRadius
    C(1), C(2), C(3), D(1), D(2), D(3), edgeRadius
    C(1), C(2), C(3), A(1), A(2), A(3), edgeRadius
    p1(1), p1(2), p1(3), p1(1), p1(2), p1(3)+1, pillarRadius
    p2(1), p2(2), p2(3), p2(1), p2(2), p2(3)+1, pillarRadius
    p3(1), p3(2), p3(3), p3(1), p3(2), p3(3)+1, pillarRadius
    p4(1), p4(2), p4(3), p4(1), p4(2), p4(3)+1, pillarRadius
    b1(1), b1(2), b1(3), b1(1), b1(2), b1(3)+0.3, bottleradius
    ptsAB(1,1),ptsAB(1,2),ptsAB(1,3)-0.03, ptsCD(1,1), ptsCD(1,2), ptsCD(1,3)-0.03, edgeRadius
    ptsAB(2,1),ptsAB(2,2),ptsAB(2,3)-0.03, ptsCD(2,1), ptsCD(2,2), ptsCD(2,3)-0.03, edgeRadius
    ptsAB(3,1),ptsAB(3,2),ptsAB(3,3)-0.03, ptsCD(3,1), ptsCD(3,2), ptsCD(3,3)-0.03, edgeRadius
    ptsAB(4,1),ptsAB(4,2),ptsAB(4,3)-0.03, ptsCD(4,1), ptsCD(4,2), ptsCD(4,3)-0.03, edgeRadius
    ptsAB(5,1),ptsAB(5,2),ptsAB(5,3)-0.03, ptsCD(5,1), ptsCD(5,2), ptsCD(5,3)-0.03, edgeRadius
    ptsAB(6,1),ptsAB(6,2),ptsAB(6,3)-0.03, ptsCD(6,1), ptsCD(6,2), ptsCD(6,3)-0.03, edgeRadius
    ptsAB(7,1),ptsAB(7,2),ptsAB(7,3)-0.03, ptsCD(7,1), ptsCD(7,2), ptsCD(7,3)-0.03, edgeRadius
    ptsAB(8,1),ptsAB(8,2),ptsAB(8,3)-0.03, ptsCD(8,1), ptsCD(8,2), ptsCD(8,3)-0.03, edgeRadius
    ptsAB(9,1),ptsAB(9,2),ptsAB(9,3)-0.03, ptsCD(9,1), ptsCD(9,2), ptsCD(9,3)-0.03, edgeRadius
    ptsAB(10,1),ptsAB(10,2),ptsAB(10,3)-0.03, ptsCD(10,1), ptsCD(10,2), ptsCD(10,3)-0.03, edgeRadius
];

%% 3. Plan with potential field
field_iterations=0;
step  = 0.02;
ok = 0;
while ok == 0
[traj, ok] = PotentialField(C_ini, C_goal, Obs);
    if ok==1
        break;
    end
field_iterations=field_iterations+1;
    if field_iterations > 50
        error('Potential field planner failed to find a valid path.1');
    end
  C_ini = C_ini + 0.05 * step * randn(1, 6);
end

if ok ~= 1
    error('Potential field planner failed to find a valid path.');
end

fprintf('Planned trajectory length: %d configurations\n', size(traj, 1));

%% 4. Animate the trajectory
Draw(traj);

%% 5. Export as URScript
scriptFile = fullfile(projectFolder, 'potential_field_trajectory.script');
fid = fopen(scriptFile, 'w');
if fid == -1
    error('Could not create script file.');
end

for i = 1:size(traj, 1)
    q = traj(i,:);
    fprintf(fid, 'movej([%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], a=1.39, v=1.04)\n', ...
        q(1), q(2), q(3), q(4), q(5), q(6));
end

fclose(fid);

fprintf('URScript file saved to: %s\n', scriptFile);
fprintf('Load this file onto the UR5e controller to execute the trajectory.\n');
