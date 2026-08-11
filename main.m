clear; clc; close all;

C_ini = [-1.1781, -1.57, -1.57, -1.57, 1.57, 0];
C_goal = [-1.08, -2.09, -1.32, -1.29, 1.55, 0.2]; % Hand-picked joint angles

Obs = []; 
[path, smooth_path] = AlexMPExtendRRT(C_ini, C_goal, Obs);

