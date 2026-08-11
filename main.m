clear; clc; close all;
ur5_kin = UR5Kinematics;

C_ini = [0, -1.5, 0, -1.5, 0, 0];

%coordinates of the goal
Goal_cords_r=[0.100; -0.700 ; -0.300];

%rotation of the TCP
theta_x=0.4;
theta_y=-0.2;
theta_z=-5.5;
    

rx=[1 0 0 ; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
ry=[cos(theta_y) 0 sin(theta_y); 0 1 0 ; -sin(theta_y) 0 cos(theta_y)];
rz=[cos(theta_z) -sin(theta_z) 0 ; sin(theta_z) cos(theta_z) 0; 0 0 1];

%rotation matrix and transformation matrix
R=rx*ry*rz;
T_be=[R Goal_cords_r; 0 0 0 1];

%multiple join angles
joint_angles=UR5Kinematics.inverse_kinematics(T_be, 0);

%obstacle collision checking for the goal joint angles and pick one for the
%rrt to find a path to
all_obstacles

for i = 1:size(joint_angles, 1)
    P = joint_angles(i, :);   % current row = one joint solution
    collision_flag=UR5_collision_checking_Obs(ur5_kin, P, Obs);
    if collision_flag==0
        break
    end
end
disp(P)
[path, smooth_path] = MPExtendRRT(C_ini, P, Obs);