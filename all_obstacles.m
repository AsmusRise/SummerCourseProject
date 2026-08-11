
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

%adding floor between the end lines:
nPoints = 10;
ptsAB = ( (1:nPoints)'/ (nPoints+1) ) .* (B - A)' + A'; % nPoints x 3
ptsCD = ( (1:nPoints)'/ (nPoints+1) ) .* (D - C)' + C'; % nPoints x 3

Obs = [
    A(1),  A(2),  A(3),  B(1),  B(2),  B(3),  edgeRadius
    B(1),  B(2),  B(3),  C(1),  C(2),  C(3),  edgeRadius
    C(1),  C(2),  C(3),  D(1),  D(2),  D(3),  edgeRadius
    D(1),  D(2),  D(3),  A(1),  A(2),  A(3),  edgeRadius
    p1(1), p1(2), p1(3), p1(1), p1(2), p1(3)+1, pillarRadius
    p2(1), p2(2), p2(3), p2(1), p2(2), p2(3)+1, pillarRadius
    p3(1), p3(2), p3(3), p3(1), p3(2), p3(3)+1, pillarRadius
    p4(1), p4(2), p4(3), p4(1), p4(2), p4(3)+1, pillarRadius

    %floor
    ptsAB(1,1),ptsAB(1,2),ptsAB(1,3), ptsCD(1,1), ptsCD(1,2), ptsCD(1,3), edgeRadius
    ptsAB(2,1),ptsAB(2,2),ptsAB(2,3), ptsCD(2,1), ptsCD(2,2), ptsCD(2,3), edgeRadius
    ptsAB(3,1),ptsAB(3,2),ptsAB(3,3), ptsCD(3,1), ptsCD(3,2), ptsCD(3,3), edgeRadius
    ptsAB(4,1),ptsAB(4,2),ptsAB(4,3), ptsCD(4,1), ptsCD(4,2), ptsCD(4,3), edgeRadius
    ptsAB(5,1),ptsAB(5,2),ptsAB(5,3), ptsCD(5,1), ptsCD(5,2), ptsCD(5,3), edgeRadius
    ptsAB(6,1),ptsAB(6,2),ptsAB(6,3), ptsCD(6,1), ptsCD(6,2), ptsCD(6,3), edgeRadius
    ptsAB(7,1),ptsAB(7,2),ptsAB(7,3), ptsCD(7,1), ptsCD(7,2), ptsCD(7,3), edgeRadius
    ptsAB(8,1),ptsAB(8,2),ptsAB(8,3), ptsCD(8,1), ptsCD(8,2), ptsCD(8,3), edgeRadius
    ptsAB(9,1),ptsAB(9,2),ptsAB(9,3), ptsCD(9,1), ptsCD(9,2), ptsCD(9,3), edgeRadius
    ptsAB(10,1),ptsAB(10,2),ptsAB(10,3), ptsCD(10,1), ptsCD(10,2), ptsCD(10,3), edgeRadius
];