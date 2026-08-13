% Potential field Project


% Take initial configuration, goal, and obstacle matrix as inputs
% Output the raw discrete path  JointTrajectory and a smoothed version JointTrajectory_smooth
function [JointTrajectory, JointTrajectory_smooth] = PotentialField2(C_ini, C_goal, Obs)
    global params;
    ParaInitialize(C_ini, C_goal, Obs);
    % Set limit of iterations to stop infinite loops when algorithm is stuck 
    params.maxiteration = 15000; 
    planningTic = tic;
    
    % Force initial and goal states to 1x6 row matrix to be sure matrix math is correct later
    q = reshape(C_ini, 1, 6);
    C_goal = reshape(C_goal, 1, 6);
    JointTrajectory = q;
    JointTrajectory_smooth = [];
    

    % tunning parameters 
    k_att = 2.0; % Scaling factor for attractive potential pulling the robot to goal 
    k_rep = 5.0; % scaling factor for repulsive potential pushing roobt away form obstacle 
    d0    = 0.10;% Range of infuence, if the robot is further than 10cm form obstacle repulsibe force is zero
    step  = 0.01;% Step size for the discrete gradient solver

    
    % State Variables for Random Walk Recovery esccaping the minima 
    recovery_mode = false;
    recovery_steps = 0;
    recovery_dir = zeros(1, 6);
    collision_fails = 0;
    best_dist = inf;
    stuck_iterations = 0;
    goalReached = false;
    recovery_activations = 0;

% Gradient Descent loop and recovery logic
% Loop runs until the robot reaches goal or hit the limit of iterations
    for it = 1:params.maxiteration % Begin the main gradient descent loop, running up to maxiteration times
        params.robot = q;
        if HasRobotReachedGoal()
            goalReached = true;
            break;
        end
        
        % Track progress measuring distance to the goal if not improved or get stuck increse iteration 
        dist_to_goal = norm(C_goal - q); % Calculate the Euclidean distance between the current configuration and the goal
        if dist_to_goal < best_dist - 0.005
            best_dist = dist_to_goal;
            stuck_iterations = 0;
        else
            stuck_iterations = stuck_iterations + 1;
        end
        
        % If the number of iterations is more than 50 trigger recovery mode 
        if stuck_iterations > 50 || collision_fails > 3
            recovery_mode = true;
            recovery_steps = 50; 
            rand_vec = randn(1, 6); 
            recovery_dir = rand_vec / norm(rand_vec); 
            stuck_iterations = 0;
            collision_fails = 0;
            recovery_activations = recovery_activations + 1;
        end
        % Move in random direction for 50 setps ignoring standard potential forces
        if recovery_mode  
            F = recovery_dir;
            recovery_steps = recovery_steps - 1;
            if recovery_steps <= 0
                recovery_mode = false;
            end
        else
        
           %  Generates a parabolic atttractive potential in  the joint space,
            F = k_att * (C_goal - q);  % Force is negative gradient of that potential
            
            % Workspace mapping, obstacles are in 3D caresian workspace, collsion avoidance is calcualted
            fk = params.ur5_kin.forward_kinematics(q); % Find exact XYZ coordinates of end effector and elbow
            p_ee = reshape(fk.transform_matrices.T6(1:3, 4), 1, 3); % Extracting the exact 3D position of the end-effector
            p_elbow = reshape(fk.transform_matrices.T3(1:3, 4), 1, 3);  % Extracting the exact 3D position of the elbow
            
            % Numerical Jacobians, using numerical approximation and pertubing each joint by tiny amount, calcualte new position
            J_v_ee = zeros(3, 6); % Initialize an empty 3x6 Jacobian matrix for the end-effector
            delta_q = 1e-5; 
            J_v_elbow = zeros(3, 6); % Initialize an empty 3x6 Jacobian matrix for the elbow
            for j = 1:6
                q_plus = q; 
                q_plus(j) = q_plus(j) + delta_q;
                fk_plus = params.ur5_kin.forward_kinematics(q_plus);
                
                p_plus_ee = reshape(fk_plus.transform_matrices.T6(1:3, 4), 1, 3);
                J_v_ee(:, j) = (p_plus_ee - p_ee)' / delta_q;
                
                p_plus_elbow = reshape(fk_plus.transform_matrices.T3(1:3, 4), 1, 3);
                J_v_elbow(:, j) = (p_plus_elbow - p_elbow)' / delta_q;
            end
            
            % Repulsive force calcualtion
            control_points = [p_ee; p_elbow]; % Group the end-effector and elbow positions into a single variable
            F_rep_cart = zeros(2, 3); 
            for pt_idx = 1:2
                p_curr = control_points(pt_idx, :); 
                % Loop through every obstacle defined in the environment
                for i = 1:size(Obs, 1)
                    a = reshape(Obs(i, 1:3), 1, 3); 
                    b = reshape(Obs(i, 4:6), 1, 3); 
                    r = Obs(i,7);
                    
                    u = b - a; 
                    uu = dot(u, u);
                    if uu < 1e-12
                        c = a; 
                    else
                        t = max(0, min(1, dot(p_curr-a, u) / uu)); 
                        c = a + t * u; 
                    end
                    
                    d = norm(p_curr - c) - r;
                    if d < d0 % If this distance is less than the influence threshold (d0), it generates a repulsive force
                        v = p_curr - c; 
                        nv = norm(v);
                        if nv < 1e-9; v = [1 0 0]; nv = 1; end
                        
                        d_safe = max(d, 0.01); 
                        
                        % Calculate force
                        rep = k_rep * (1 / d_safe - 1 / d0) * (1 / d_safe^2) * (v / nv);
                        
                        % GNRON FIX: Multiply by distance to goal squared
                        rep = rep * (dist_to_goal^2);
                        
                        F_rep_cart(pt_idx, :) = F_rep_cart(pt_idx, :) + rep; 
                    end
                end
            end
            
            %  Convert cartesian repulsive forces applied to elbow and end effector to join space torques 
            F_rep_joint_ee = (J_v_ee' * F_rep_cart(1, :)')'; 
            F_rep_joint_elbow = (J_v_elbow' * F_rep_cart(2, :)')'; 
            F = F + F_rep_joint_ee + F_rep_joint_elbow; % Add them to precalculated join space attractive force
            
            nF = norm(F); %  Normalize combined force vector turning to pure direction vector
            if nF > 1e-6
                F = F / nF; 
            end
        end
        
        % Apply step with joint limits clamping
        qnew = q + step * F; % Move robot by one dicrete step in the direction of combined force field
        qmin = [params.Q1min, params.Q2min, params.Q3min, params.Q4min, params.Q5min, params.Q6min];
        qmax = [params.Q1max, params.Q2max, params.Q3max, params.Q4max, params.Q5max, params.Q6max];
        qnew = min(max(qnew, qmin), qmax); % Cap the join angles form breaking physical limits of the UR5 arm
        
        % Collision checking on updated state to verify that new position does not touch any obstacle 
       % If it touches discad the point and increment collision_fail by one ( trigger random walk recovery)
        params.robot = qnew;
        if IsValidState() == 1
            q = qnew;
            JointTrajectory = [JointTrajectory; q]; 
        else
            collision_fails = collision_fails + 1;
        end
    end

    % If goal is reached pass it to smoothing function removing unnecessary control point before final result
    if goalReached
        if exist('SmoothPath', 'file') == 2
            JointTrajectory_smooth = SmoothPath(JointTrajectory);
        else
            JointTrajectory_smooth = JointTrajectory;
        end
        planningSeconds = toc(planningTic);
    else
        JointTrajectory = [];
        JointTrajectory_smooth = [];
        planningSeconds = toc(planningTic);
    end

    logAPFRun(C_ini, C_goal, it, JointTrajectory, JointTrajectory_smooth, goalReached, planningSeconds, recovery_activations, best_dist, collision_fails);
end

function logAPFRun(C_ini, C_goal, iter, JointTrajectory, JointTrajectory_smooth, successFlag, planningSeconds, recoveryActivations, bestDist, collisionFails)
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
fprintf(logFid, 'Path configurations: %d\n', size(JointTrajectory, 1));
fprintf(logFid, 'Smoothed path configurations: %d\n', size(JointTrajectory_smooth, 1));
fprintf(logFid, 'moveJ count: %d\n', moveJCount);
fprintf(logFid, 'Goal reached: %d\n', successFlag);
fprintf(logFid, 'Recovery activations: %d\n', recoveryActivations);
fprintf(logFid, 'Best distance to goal: %.6f\n', bestDist);
fprintf(logFid, 'Collision failures: %d\n', collisionFails);
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