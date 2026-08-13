clear; clc; close all;

C_ini = [0, -1.5, 0, -1.5, 0, 0];
C_goal = [0.3, -1.2, -0.9, -1.2, 1.0, 0]; % Hand-picked joint angles

Obs = []; 
[path, smooth_path] = MPExtendRRT(C_ini, C_goal, Obs);