% Potential field planner tailored to this UR5 project.
% Inputs: C_ini, C_goal, Obs
% Output: JointTrajectory and success flag

function [JointTrajectory, status] = PotentialField(C_ini, C_goal, Obs)

    global params;

    ParaInitialize(C_ini, C_goal, Obs);

    q = C_ini(:).';
    JointTrajectory = q;

    k_att = 1.0;
    k_rep = 80;
    d0    = 0.25;
    step  = 0.02;

    for it = 1:params.maxiteration
        params.robot = q;

        if HasRobotReachedGoal()
            status = 1;
            return;
        end

        F = k_att * (C_goal - q);

        % Use the first three joints as a simplified workspace point.
        p = q(1:3);

        for i = 1:size(Obs, 1)
            a = Obs(i,1:3);
            b = Obs(i,4:6);
            r = Obs(i,7);
            u = b - a;
            uu = dot(u, u);

            if uu < 1e-12
                c = a;
            else
                t = max(0, min(1, dot(p-a, u) / uu));
                c = a + t * u;
            end

            d = norm(p - c) - r;
            if d < d0
                v = p - c;
                nv = norm(v);
                if nv < 1e-9
                    v = [1 0 0];
                    nv = 1;
                end

                rep = k_rep * (1 / max(d, 1e-6) - 1 / d0) * ...
                    (1 / max(d, 1e-6)^2) * (v / nv);
                F(1:3) = F(1:3) + rep;
            end
        end

        nF = norm(F);
        if nF < 1e-4
            status = 0;
            return;
        end

        qnew = q + step * F / nF;

        qnew = min(max(qnew, [params.Q1min params.Q2min params.Q3min params.Q4min params.Q5min params.Q6min]), ...
                            [params.Q1max params.Q2max params.Q3max params.Q4max params.Q5max params.Q6max]);

        params.robot = qnew;
        if IsValidState()
            q = qnew;
            JointTrajectory = [JointTrajectory; q];
        else
            q = q + 0.05 * step * randn(1, 6);
        end
    end

    status = 0;
end