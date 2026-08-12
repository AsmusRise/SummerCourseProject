%% UR5 RRT motion-planning runner
% Add obstacles in Obs. The RRT planning, smoothing, drawing, saving,
% and URScript export are done automatically.

clear
clc
close all

% Make all project functions available
projectFolder = fileparts(mfilename('fullpath'));
addpath(genpath(projectFolder));

% Same random result each time; remove/change this for a different path
rng(1);
showSimulation = 0;
%% Angles for chess pieces and drop off point
%cp1 is left bottom corner
C_cp1_up = [deg2rad(-11.76),deg2rad(-77.6), deg2rad(-121.7),deg2rad(-70.6),deg2rad(89.6),deg2rad(11)]; %the position when it needs to open its gripper
C_cp1_down = [deg2rad(-11.68),deg2rad(-84.43), deg2rad(-133.36),deg2rad(-50.10),deg2rad(90.28),deg2rad(10.96)]; %position where it can pick it up.
%cp2 is the bottom right corner piece
C_cp2_up = [deg2rad(-102.66),deg2rad(-76.69), deg2rad(-115.63),deg2rad(-78.46),deg2rad(90.4),deg2rad(10.95)];
C_cp2_down = [deg2rad(-102.72),deg2rad(-85.21), deg2rad(-135.75),deg2rad(-44.37),deg2rad(91.3),deg2rad(10.96)];
%cp3 is the top right corner piece
C_cp3_up = [deg2rad(-78.09),deg2rad(-141.93), deg2rad(-30.73),deg2rad(-96.23),deg2rad(89.12),deg2rad(11)];
C_cp3_down = [deg2rad(-81.27),deg2rad(-144.24), deg2rad(-42.88),deg2rad(-90.13),deg2rad(88.94),deg2rad(1.42)];
%drop off position
C_dop = [deg2rad(-45.4),deg2rad(-128.73), deg2rad(-43.04),deg2rad(-83.4),deg2rad(91.96),deg2rad(1.51)];

%% 1. Start and goal joint configurations (radians)
C_ini  = C_cp1_up;
C_goal = C_cp2_up;

%% 2. Obstacles
% One row per capsule:
% [x_start y_start z_start  x_end y_end z_end  radius]
%
% Add more obstacles by adding more rows below.

% Measurements are given in millimetres. Convert to meters for the planner.
mm2m = 1e-3;

% Robot-frame points.
A = [242; 400; -30] * mm2m; %closest to corner
B = [-493; 90; -34] * mm2m; %down at the bottom aswell
C = [685; -707; -28] * mm2m; %corner but close to pillar
D = [-38; -1014; -26] * mm2m;

p1 = [568; -394; -30] * mm2m; %left pillars
p2 = [605; -481; -23] * mm2m;
p3 = [-175; -690; -28] * mm2m; %right pillars
p4 = [-144; -783; -30] * mm2m;

b1 = [177; -495.6; -15] * mm2m; %bottle

%Chess pieces pickup coordinates:
cp1 =  [347, -183.7, -15] * mm2m; % left bottom
cp2 = [-183, -346.5, -15] * mm2m; % right bottom
cp3 = [72.7, -838.6, -15] * mm2m; % right top

%boxCorners
bA = [605.62, -488, 123] * mm2m;
bB = [388.81, -568.84, 120.4] * mm2m;
bC = [466,-773, 125] * mm2m;

%drop off point
dop =  [490, -628, 201] * mm2m;


%floor % Create 10 points along edge A->B and 10 along C->D
nPoints = 10;
ptsAB = ( (1:nPoints)'/ (nPoints+1) ) .* (B - A)' + A'; % nPoints x 3
ptsCD = ( (1:nPoints)'/ (nPoints+1) ) .* (D - C)' + C'; % nPoints x 3


% Optional table-frame rotation from your notes.
theta = 22.5 * (pi / 180);
R_ab = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

% Rotated coordinates in the table frame, kept here for reference.
A_t = R_ab * A;
B_t = R_ab * B;
C_t = R_ab * C;
D_t = R_ab * D;
p1_t = R_ab * p1;
p2_t = R_ab * p2;
p3_t = R_ab * p3;
p4_t = R_ab * p4;

% Use the robot-frame geometry for collision checking.
edgeRadius = 0.02;

pillarRadius = 0.03;
pillarRadiusThick = 0.05;
bottleradius = 0.04;



Obs = [
    A(1),  A(2),  A(3),  B(1),  B(2),  B(3),  edgeRadius
    B(1),  B(2),  B(3),  D(1),  D(2),  D(3),  edgeRadius
    C(1),  C(2),  C(3),  D(1),  D(2),  D(3),  edgeRadius
    C(1),  C(2),  C(3),  A(1),  A(2),  A(3),  edgeRadius
    p1(1), p1(2), p1(3), p1(1), p1(2), p1(3)+1, pillarRadius
    p2(1), p2(2), p2(3), p2(1), p2(2), p2(3)+1, pillarRadius
    p3(1), p3(2), p3(3), p3(1), p3(2), p3(3)+1, pillarRadiusThick
    p4(1), p4(2), p4(3), p4(1), p4(2), p4(3)+1, pillarRadiusThick

    %bottle 
    b1(1), b1(2),b1(3),b1(1), b1(2),b1(3)+0.27,bottleradius
];

BoxDropOff = [
    bA(1),  bA(2),  bA(3),  bB(1),  bB(2),  bB(3),  edgeRadius
    bB(1),  bB(2),  bB(3),  bC(1),  bC(2),  bC(3),  edgeRadius
    ];

Floor = [ptsAB(1,1),ptsAB(1,2),ptsAB(1,3)-0.03, ptsCD(1,1), ptsCD(1,2), ptsCD(1,3)-0.03, edgeRadius
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
Obs = [Obs;BoxDropOff;Floor];


fprintf('Obstacle set uses %d capsule segments.\n', size(Obs, 1));
global params
params.showSimulation = showSimulation;
ParaInitialize(C_ini, C_goal, Obs)

params.logFile = fullfile(projectFolder, 'rrt_run_log.txt');
logResetFid = fopen(params.logFile, 'w');
if logResetFid == -1
    error('Could not reset the log file.');
end
fprintf(logResetFid, 'UR5 RRT run log\n');
fprintf(logResetFid, 'Run started: %s\n', datestr(now));
fprintf(logResetFid, '---\n');
fclose(logResetFid);

params.robot = C_ini;
startWithinLimits = params.robot(1) >= params.Q1min && params.robot(1) <= params.Q1max && ...
    params.robot(2) >= params.Q2min && params.robot(2) <= params.Q2max && ...
    params.robot(3) >= params.Q3min && params.robot(3) <= params.Q3max && ...
    params.robot(4) >= params.Q4min && params.robot(4) <= params.Q4max && ...
    params.robot(5) >= params.Q5min && params.robot(5) <= params.Q5max && ...
    params.robot(6) >= params.Q6min && params.robot(6) <= params.Q6max;
if ~startWithinLimits
    fprintf('Start configuration violates joint limits: %s\n', mat2str(C_ini, 4));
    fprintf('Q1 allowed range is [%.4f, %.4f] rad, but start Q1 = %.4f rad\n', params.Q1min, params.Q1max, C_ini(1));
    error('Fix C_ini before planning.');
end
if IsValidState() == 0
    error('Start configuration is inside collision space.');
else
    fprintf('Start configuration is valid.\n');
end

for obsIdx = 1:size(Obs, 1)
    collisionWithObs = UR5_collision_checking(params.ur5_kin, params.robot, Obs(obsIdx,1:3), Obs(obsIdx,4:6), Obs(obsIdx,7));
    if collisionWithObs == 1
        fprintf('Start configuration collides with obstacle %d: [%.3f %.3f %.3f -> %.3f %.3f %.3f, r=%.3f]\n', ...
            obsIdx, Obs(obsIdx,1), Obs(obsIdx,2), Obs(obsIdx,3), Obs(obsIdx,4), Obs(obsIdx,5), Obs(obsIdx,6), Obs(obsIdx,7));
    end
end

params.robot = C_goal;
goalWithinLimits = params.robot(1) >= params.Q1min && params.robot(1) <= params.Q1max && ...
    params.robot(2) >= params.Q2min && params.robot(2) <= params.Q2max && ...
    params.robot(3) >= params.Q3min && params.robot(3) <= params.Q3max && ...
    params.robot(4) >= params.Q4min && params.robot(4) <= params.Q4max && ...
    params.robot(5) >= params.Q5min && params.robot(5) <= params.Q5max && ...
    params.robot(6) >= params.Q6min && params.robot(6) <= params.Q6max;
if ~goalWithinLimits
    fprintf('Goal configuration violates joint limits: %s\n', mat2str(rad2deg(C_goal), 4));
    error('Fix C_goal before planning.');
end
if IsValidState() == 0
    error('Goal configuration is inside collision space.');
else
    fprintf('Goal configuration is valid.\n');
end

for obsIdx = 1:size(Obs, 1)
    collisionWithObs = UR5_collision_checking(params.ur5_kin, params.robot, Obs(obsIdx,1:3), Obs(obsIdx,4:6), Obs(obsIdx,7));
    if collisionWithObs == 1
        fprintf('Goal configuration collides with obstacle %d: [%.3f %.3f %.3f -> %.3f %.3f %.3f, r=%.3f]\n', ...
            obsIdx, Obs(obsIdx,1), Obs(obsIdx,2), Obs(obsIdx,3), Obs(obsIdx,4), Obs(obsIdx,5), Obs(obsIdx,6), Obs(obsIdx,7));
    end
end
%% 3. Plan, smooth, and draw the motion
[path, smoothPath] = MPExtendRRT(C_ini, C_goal, Obs);

%% 4. Check result
if isempty(path)
    error(['No collision-free path was found. ' ...
        'Try moving/reducing the obstacle or increasing maxiteration.']);
end

fprintf('Raw RRT path: %d configurations\n', size(path, 1));
fprintf('Smoothed path: %d configurations\n', size(smoothPath, 1));
fprintf('Goal error: %.4f rad\n', norm(smoothPath(end,:) - C_goal));

%% 5. Save MATLAB result
save(fullfile(projectFolder, 'rrt_result.mat'), ...
    'C_ini', 'C_goal', 'Obs', 'path', 'smoothPath');

%% 6. Export the smooth path as URScript
scriptFile = fullfile(projectFolder, 'rrt_trajectory.script');
fid = fopen(scriptFile, 'w');

if fid == -1
    error('Could not create the URScript file.');
end

for i = 1:size(smoothPath, 1)
    q = smoothPath(i,:);
    fprintf(fid, ...
        'movej([%.6e,%.6e,%.6e,%.6e,%.6e,%.6e],a=1.39,v=1.04)\n', ...
        q(1), q(2), q(3), q(4), q(5), q(6));
end

fclose(fid);

fprintf('\nDone.\n');
fprintf('MATLAB result: %s\n', fullfile(projectFolder, 'rrt_result.mat'));
fprintf('URScript file: %s\n', scriptFile);