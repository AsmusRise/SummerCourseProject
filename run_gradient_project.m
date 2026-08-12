%% UR5 Gradient-descent motion-planning runner
% Add obstacles in Obs. The gradient-descent planning, smoothing, drawing,
% saving, and URScript export are done automatically.

clear
clc
close all

% Make all project functions available
projectFolder = fileparts(mfilename('fullpath'));
addpath(genpath(projectFolder));

% Same random result each time for repeatability
rng(1);

%% 1. Start and goal joint configurations (radians)
C_ini  = [0, -pi/2, 0, 0, 0, 0];
C_goal = [0.8, -1.1, 0.7, 0.4, -0.4, 0.3];

%% 2. Obstacles
% One row per capsule:
% [x_start y_start z_start  x_end y_end z_end  radius]
%
% Add more obstacles by adding more rows below.

%% Ground-level frame centered around the UR5 base
groundZ = 0;
r = 0.03;
z = groundZ + r;

% Square frame around the robot base; the centre remains free
baseCapsules = [
    -0.35, -0.35, z,   0.35, -0.35, z,   r   % front edge
    -0.35,  0.35, z,   0.35,  0.35, z,   r   % back edge
    -0.35, -0.35, z,  -0.35,  0.35, z,   r   % left edge
     0.35, -0.35, z,   0.35,  0.35, z,   r   % right edge
];

%% Two vertical round pillars on the base frame
pillarRadius = 0.05;
pillarHeight = 0.40;

pillar1 = [-0.25, 0.25, pillarRadius, ...
           -0.25, 0.25, pillarHeight-pillarRadius, pillarRadius];

pillar2 = [ 0.25, 0.25, pillarRadius, ...
            0.25, 0.25, pillarHeight-pillarRadius, pillarRadius];

Obs = [
    baseCapsules
    pillar1
    pillar2
];
global params
ParaInitialize(C_ini, C_goal, Obs)

params.robot = C_ini;
assert(IsValidState() == 1, 'Start configuration collides with an obstacle.')

params.robot = C_goal;
assert(IsValidState() == 1, 'Goal configuration collides with an obstacle.')

%% 3. Plan, smooth, and draw the motion
[path, smoothPath] = MPPotentialField(C_ini, C_goal, Obs);

%% 4. Check result
if isempty(path)
    error(['No collision-free path was found. ' ...
        'Try moving/reducing the obstacle or adjusting the gradient-descent parameters.']);
end

fprintf('Raw gradient path: %d configurations\n', size(path, 1));
fprintf('Smoothed path: %d configurations\n', size(smoothPath, 1));
fprintf('Goal error: %.4f rad\n', norm(smoothPath(end,:) - C_goal));

%% 5. Save MATLAB result
save(fullfile(projectFolder, 'gradient_result.mat'), ...
    'C_ini', 'C_goal', 'Obs', 'path', 'smoothPath');

%% 6. Export the smooth path as URScript
scriptFile = fullfile(projectFolder, 'gradient_trajectory.script');
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
printf('MATLAB result: %s\n', fullfile(projectFolder, 'gradient_result.mat'));
printf('URScript file: %s\n', scriptFile);
