clear; clc; close all;
ur5_kin = UR5Kinematics;


%initial position
C_ini = [-1.178, -1.57, -1.57, -1.57, 1.57, 0];

%coordinates of the TCP in mm
x=220;
y=-670;
z=18;
%rotation of the TCP
theta_x=0.7;
theta_y=-3.018;
theta_z=-0.013;

%weights for route selection
C_weights=[2 0.2 0.2 1 2 2];



x_m= x/1000;
y_m= y/1000;
z_m= z/1000;
Goal_cords_r=[x_m; y_m ; z_m];

rx=[1 0 0 ; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
ry=[cos(theta_y) 0 sin(theta_y); 0 1 0 ; -sin(theta_y) 0 cos(theta_y)];
rz=[cos(theta_z) -sin(theta_z) 0 ; sin(theta_z) cos(theta_z) 0; 0 0 1];

%rotation matrix and transformation matrix
R=rz*ry*rx;
T_be=[R Goal_cords_r; 0 0 0 1];

%multiple join angles
joint_angles=UR5Kinematics.inverse_kinematics(T_be, 0);
disp(joint_angles)
%obstacle collision checking for the goal joint angles and pick one for the
%rrt to find a path to
all_obstacles
smallest_diff=1000;
for i = 1:size(joint_angles, 1)
    C_goal = joint_angles(i, :);   % current row = one joint solution
    collision_flag=UR5_collision_checking_Obs(ur5_kin, C_goal, Obs);
    fprintf('Row %d: collision_flag = %d\n', i, collision_flag);
    if collision_flag==0
        %if collision is zero check for angle distance
         C_ini_deg=rad2deg(C_ini);
         C_goal_deg=rad2deg(C_goal);
         theta_diff=wrapToPi(C_ini-C_goal) .* C_weights;
         disp(theta_diff)
         total_diff=sum(abs(theta_diff));
        
         disp(total_diff)
         if total_diff < smallest_diff
         smallest_diff = total_diff;
         smallest_Goal=C_goal;
         end
    end
end

if smallest_diff == 1000
    fprintf('No collision-free joint angles exist\n');
else
    disp(smallest_Goal)
    rad2deg(smallest_Goal)
end
    %{
if collision_flag ~= 0
    fprintf('no collision free joint angles exist');
else
    disp(C_goal);
end
    
%[path, smooth_path] = MPExtendRRT(C_ini, P, Obs);

C_goal= rad2deg(C_goal);
joint_angles=rad2deg(joint_angles);
disp(C_goal)
disp(joint_angles)

    %}
