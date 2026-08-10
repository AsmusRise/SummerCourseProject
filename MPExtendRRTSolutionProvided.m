function [] = MPExtendRRT()
global mp;
global params;

% Implement the RRT algorithm as described in class

if rand() < 0.1
    sto = [params.goal(1), params.goal(2)];
else
    sto = SampleState();
end

dmin = Inf;
vid    = -1;
n      = length(mp.xpts);
for k = 1 : 1 : n
    d = (sto(1) - mp.xpts(k))^2 + (sto(2) - mp.ypts(k))^2;
    if d < dmin
        dmin = d;
        vid    = k;
    end
end

MPExtendTree(vid, sto);
end

