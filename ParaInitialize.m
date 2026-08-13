function [  ] = ParaInitialize(C_ini, C_goal, Obs)

% this function is used to initialize the struct params

global params;

params.robot        = C_ini;
params.goal         = C_goal;
params.obstacles    = Obs;
params.distOneStep  = 0.1;              % the step size in MPExtendTree
params.goaltolerance= 0.05;             % the goal tolerance is used to check if the configuration is close enough to the goal configuration
params.maxiteration = 10000;            % the maximum iterations number for the algorithm
params.smoothiters  = 200;              % the maximum iternation number for the post-processing in SmoothPath
params.ur5_kin      = UR5Kinematics();

% Joint limits of all the six joints of UR5

% Important: please start with these joint limits, if experiencing
% self-collision or insufficient motion range, you can change the joint
% limits carefully.

% please note that there are hard joint limits for the real UR5 robot,
% which is from -360 to -360 degree for all the joints. 

% CAVEAT: using full motion ranges of all the joints may cause self-collision. 
% Self-collision is inplemented by the joint limits below, it is not
% included in the collision checking functions, UR5_collision_checking and UR5_collision_checking_Obs

params.Q1min        = (-67.5 - 90) * pi / 180;
params.Q1max        = (-67.5 + 90) * pi / 180;
params.Q2min        = (-90 - 60) * pi / 180;
params.Q2max        = (-90 + 60) * pi / 180;
params.Q3min        = -137 * pi / 180;
params.Q3max        = 90 * pi / 180;
params.Q4min        = -100 * pi / 180;
params.Q4max        = 100 * pi / 180;
params.Q5min        = -90 * pi / 180;
params.Q5max        = 92 * pi / 180;
params.Q6min        = -90 * pi / 180;
params.Q6max        = 90 * pi / 180;

end

