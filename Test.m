figure
% model = createpde;
% gm1 = importGeometry(model, 'Clover_Assembly - Arm-1.STL');
model2= createpde;
gm2 = importGeometry(model2, 'ParrotQuadrotor.STL');
% pdegplot(gm1)
hold on
pdegplot(gm2)
hold on
% plot([0 10], [20 30])
% translate(gm1, [0 0 0]);
% gm1.rotate(45)
% rotate(gm1,[0, 90]);
h = rotate(gm2,45,[0 0 0]);
h1 = translate(gm2, [2 2 2]);
pdegplot(gm2)
% translate(gm2, [0 0 0]);
%%
% ply = pcread('block_ply.PLY');
% pcshow(ply)

%%
PlaceObject('CloverAssembly.PLY');
%%
figure(2)
PlaceObject('Clover2.PLY');

%%
figure(3)
axis tight
axis equal
Clover = PlaceObj('CloverAssemblyP.PLY');

Clover.MoveObj([0 0 0 0 0 0]);
%%
Clover.MoveObj([0 0 0 pi/2 0 0]);
%%
camlight
%%
clc
addpath(genpath('./Addition'));
quadParams = ReadProperty("CloverProp.pdf");
initState = zeros(12,1);
initInput = zeros(4,1);
simTime = 20;
quad = Quadcopter(quadParams, initState, initInput, simTime);
% quad.Model3D('CloverAssemblyP.PLY');
% t = quad.T

% Trajectory
t = 0:0.01:simTime;
x = 0.5*cos(0.1*t);
dx = -0.05*sin(0.1*t);
d2x = -0.005*cos(0.1*t);
y = 0.5*sin(0.1*t);
dy = 0.05*cos(0.1*t);
d2y = -0.005*sin(0.1*t);
z = 0.01*t.^2;
dz = 0.04*t;
d2z = 0.04*ones(1,size(t,2));
rot = zeros(3,size(t,2));
drot = zeros(3,size(t,2));
d2rot = zeros(3,size(t,2));
for i = 1:size(t,2)
    quad.TrajectoryControl([x(i);y(i);z(i)],[dx(i);dy(i);dz(i)],[d2x(i);d2y(i);d2z(i)]);
    rot(:,i) = quad.rot;
    if i > 1
        drot(:,i) = rot(:,i) - rot(:,i-1);
        d2rot(:,i) = drot(:,i) - drot(:,i-1);
    end
    quad.EOM(drot(:,i),d2rot(:,i));
    quad.M
    i
end
% quad.EOM
% moment = quad.M

%%
clc
Coriolis
%%
a = ReadProperty("CloverProp.pdf")
a.values
a.keys
%%
syms T phi tta
m = 0.3;
g = 9.81;
dtrans = [0.1;0.2;0.3];
d2trans = [0.01;0.02;0.03];
r = RotMat([phi tta 0]).'*(m*([0;0;g]+d2trans) + 0.25*dtrans)
eq = [0;0;T] == r
range = [-pi/2 pi/2;-pi/2 pi/2;-Inf Inf];
[phiNum, ttaNum, thrustNum] = vpasolve(eq, [phi tta T])

% dx = m*d2trans(1) + 0.25*dtrans(1);
% dy = m*d2trans(2) + 0.25*dtrans(2);
% dz = d2trans(3)*m*g + 0.25*dtrans(3);
% phiN = asin( (dx*sin(0)-dy*cos(0))/ sqrt(dx.^2+dy.^2+(dz).^2) )
% ttaN = atan( (dx*cos(0)+dy*sin(0))/ ((dz)) )
% TN = m*(dx*(cos(phiN)*sin(ttaN)*cos(0)+sin(phiN)*sin(0)) + dy*(cos(phiN)*sin(ttaN)*sin(0)-sin(phiN)*cos(0)) + (dz)*cos(phiN)*cos(ttaN))

%%
syms thrust phi tta
a = [2];
dragMat = diag(0.25*ones(1,3));
rot(3) = 0;
range = [-pi/2 pi/2;-pi/2 pi/2;-Inf Inf];
eq = [0; 0; thrust] == RotMat([phi tta rot(3)]).'*(m*([0;0;g] + d2trans) + dragMat*dtrans)
[phiVal, ttaVal, thrustVal]  = vpasolve(eq, [phi tta thrust], range)




