function q = potential_field_v1(q0, q_goal, params)
    q = q0;
    alpha = params.PF_stepSize;
    tol = params.goaltolerance;
    maxIter = params.maxiteration;

    for iter = 1:maxIter
        gradU_att = params.PF_k_att * (q - q_goal);
        gradU_rep = computeRepulsiveGradient(q, params);

        gradU = gradU_att + gradU_rep;
        q = q - alpha * gradU;

        q = projectJointLimits(q, params);

        params.robot = q;
        if IsValidState() == 0
            q = q + 0.5 * alpha * randn(1,6);  % small escape perturbation
            q = projectJointLimits(q, params);
        end

        if norm(q - q_goal) < tol
            break;
        end
    end
end