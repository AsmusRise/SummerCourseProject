
mm2m= 1e-3;
%Obstacles = all_obstacles.Obs; 
A = [242; 400; -30] * mm2m; %closest to corner
B = [-493; 90; -34] * mm2m; %down at the bottom aswell
C = [685; -707; -28] * mm2m; %corner but close to pillar
D = [-38; -1014; -26] * mm2m;

p1 = [568; -394; -30] * mm2m;
p2 = [605; -481; -23] * mm2m;
p3 = [-175; -690; -28] * mm2m;
p4 = [-144; -783; -30] * mm2m;

edgeRadius = 0.02;
pillarRadius = 0.03;

Obs = [
    A(1),  A(2),  A(3),  B(1),  B(2),  B(3),  edgeRadius
    B(1),  B(2),  B(3),  C(1),  C(2),  C(3),  edgeRadius
    C(1),  C(2),  C(3),  D(1),  D(2),  D(3),  edgeRadius
    D(1),  D(2),  D(3),  A(1),  A(2),  A(3),  edgeRadius
    p1(1), p1(2), p1(3), p1(1), p1(2), p1(3)+1, pillarRadius
    p2(1), p2(2), p2(3), p2(1), p2(2), p2(3)+1, pillarRadius
    p3(1), p3(2), p3(3), p3(1), p3(2), p3(3)+1, pillarRadius
    p4(1), p4(2), p4(3), p4(1), p4(2), p4(3)+1, pillarRadius
];