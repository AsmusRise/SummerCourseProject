function [JointTrajectory, JointTrajectory_smooth] = MPPotentialField(C_ini, C_goal, Obs)
% Gradient-descent / potential-field planner in joint space for UR5.
% Returns raw and smoothed joint-space trajectories.

global params;

JointTrajectory = [];
JointTrajectory_smooth = [];

ParaInitialize(C_ini, C_goal, Obs);
params.robot = C_ini;

q = C_ini;

maxIter = params.maxiteration;
alpha = params.PF_stepSize;

path = q;

for iter = 1:maxIter
    grad_att = params.PF_k_att * (q - C_goal);
    grad_rep = computeRepulsiveGradient(q, Obs, params);
    gradU = grad_att + grad_rep;

    q_next = q - alpha * gradU;
    q_next = projectJointLimits(q_next, params);

    params.robot = q_next;
    if IsValidState() == 0
        q_next = q + 0.5 * alpha * randn(1,6);
        q_next = projectJointLimits(q_next, params);
        params.robot = q_next;
        if IsValidState() == 0
            continue;
        end
    end

    q = q_next;
    path = [path; q];

    if norm(q - C_goal) <= params.goaltolerance
        break;
    end
end

if norm(q - C_goal) <= params.goaltolerance
    JointTrajectory = path;
    JointTrajectory_smooth = SmoothPath(JointTrajectory);
    Draw(JointTrajectory_smooth);
else
    JointTrajectory = [];
    JointTrajectory_smooth = [];
end
end

function grad = computeRepulsiveGradient(q, Obs, params)
    grad = zeros(1,6);
    d0 = params.PF_repRadius;
    k_rep = params.PF_k_rep;

    for obsIdx = 1:size(Obs,1)
        d = obstacleDistance(q, Obs(obsIdx,:), params);
        if d <= 0 || d >= d0
            continue;
        end

        grad_d = finiteDifferenceGradient(q, Obs(obsIdx,:), params);
        grad = grad + k_rep * (1/d - 1/d0) * (1 / (d^2)) * grad_d;
    end
end

function d = obstacleDistance(q, obs, params)
    ur5_kin = params.ur5_kin;
    result = ur5_kin.forward_kinematics(q);
    threshold_obs = 0.01;

    offset1 = 0.12;
    offset2 = 0.115;
    offset3 = 0.075;
    offset4 = 0.025;

    L1_r = 0.040;
    L2_r = 0.035;
    L3_r = 0.035;
    L4_r = 0.035;
    L5_r = 0.025;
    L6_r = 0.025;
    L7_r = 0.025;
    L8_r = 0.025;

    L1_ini = result.transform_matrices.T1(1:3,4) + offset1 * result.transform_matrices.T1(1:3,3);
    L1_end = result.transform_matrices.T2(1:3,4) + offset2 * result.transform_matrices.T2(1:3,3);

    L2_ini = result.transform_matrices.T2(1:3,4);
    L2_end = result.transform_matrices.T3(1:3,4);

    L3_ini = result.transform_matrices.T4(1:3,4);
    L3_end = result.transform_matrices.T5(1:3,4);

    L4_ini = result.transform_matrices.T5(1:3,4);
    L4_end = result.transform_matrices.T6(1:3,4);

    L5_ini = result.transform_matrices.T6(1:3,4) + offset3 * result.transform_matrices.T6(1:3,2) + offset4 * result.transform_matrices.T6(1:3,3);
    L5_end = result.transform_matrices.T6(1:3,4) - offset3 * result.transform_matrices.T6(1:3,2) + offset4 * result.transform_matrices.T6(1:3,3);

    L6_ini = L5_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
    L6_end = L5_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

    L7_ini = L6_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
    L7_end = L6_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

    L8_ini = L7_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
    L8_end = L7_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

    L1_Obs = Dmin(L1_ini', L1_end', obs(1:3), obs(4:6)) - (L1_r + obs(7)) - threshold_obs;
    L2_Obs = Dmin(L2_ini', L2_end', obs(1:3), obs(4:6)) - (L2_r + obs(7)) - threshold_obs;
    L3_Obs = Dmin(L3_ini', L3_end', obs(1:3), obs(4:6)) - (L3_r + obs(7)) - threshold_obs;
    L4_Obs = Dmin(L4_ini', L4_end', obs(1:3), obs(4:6)) - (L4_r + obs(7)) - threshold_obs;
    L5_Obs = Dmin(L5_ini', L5_end', obs(1:3), obs(4:6)) - (L5_r + obs(7)) - threshold_obs;
    L6_Obs = Dmin(L6_ini', L6_end', obs(1:3), obs(4:6)) - (L6_r + obs(7)) - threshold_obs;
    L7_Obs = Dmin(L7_ini', L7_end', obs(1:3), obs(4:6)) - (L7_r + obs(7)) - threshold_obs;
    L8_Obs = Dmin(L8_ini', L8_end', obs(1:3), obs(4:6)) - (L8_r + obs(7)) - threshold_obs;

    minDist = min([L1_Obs, L2_Obs, L3_Obs, L4_Obs, L5_Obs, L6_Obs, L7_Obs, L8_Obs]);
    d = max(minDist, 1e-3);
end

function grad = finiteDifferenceGradient(q, obs, params)
    delta = 1e-4;
    grad = zeros(1,6);

    for j = 1:6
        dq = zeros(1,6);
        dq(j) = delta;
        d_plus = obstacleDistance(q + dq, obs, params);
        d_minus = obstacleDistance(q - dq, obs, params);
        grad(j) = (d_plus - d_minus) / (2 * delta);
    end
end

function q = projectJointLimits(q, params)
    q(1) = min(max(q(1), params.Q1min), params.Q1max);
    q(2) = min(max(q(2), params.Q2min), params.Q2max);
    q(3) = min(max(q(3), params.Q3min), params.Q3max);
    q(4) = min(max(q(4), params.Q4min), params.Q4max);
    q(5) = min(max(q(5), params.Q5min), params.Q5max);
    q(6) = min(max(q(6), params.Q6min), params.Q6max);
end
