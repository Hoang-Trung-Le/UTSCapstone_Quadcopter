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
uav = PlaceObj('CloverAssemblyP.PLY');
%%
clc
addpath(genpath('./Addition'));
quadParams = ReadProperty("CloverProp.pdf");
initState = zeros(12,1);
initInput = zeros(4,1);
simTime = 10;
quad = Quadcopter(quadParams, initState, initInput);
% quad.Model3D('CloverAssemblyP.PLY');
% t = quad.T

% Trajectory
t = 0:0.01:simTime;
amp = 0.5;
freq = pi/2;
x = amp*cos(freq*t);
dx = -amp*freq*sin(freq*t);
d2x = -amp*freq^2*cos(freq*t);
y = amp*sin(freq*t);
dy = amp*freq*cos(freq*t);
d2y = -amp*freq^2*sin(freq*t);
z = 0.02*t.^2;
dz = 0.04*t;
d2z = 0.04*ones(1,size(t,2));
rot = zeros(3,size(t,2));
drot = zeros(3,size(t,2));
d2rot = zeros(3,size(t,2));
omega = zeros(4,size(t,2));
figure(1)
hold on
axis([-1 1 -1 1 0 3])
axis tight
for i = 1:size(t,2)
    quad.TrajectoryControl([x(i);y(i);z(i)],[dx(i);dy(i);dz(i)],[d2x(i);d2y(i);d2z(i)]);
    rot(:,i) = quad.rot;
    if i > 1
        drot(:,i) = (rot(:,i) - rot(:,i-1))/0.01;
        d2rot(:,i) = (drot(:,i) - drot(:,i-1))/0.01;
    end
    quad.EOM(drot(:,i),d2rot(:,i));
%     quad.M
    quad.ControlInput
    omega(:,i) = double(quad.omega);
    uav.MoveObj([x(i) y(i) z(i) rot(1,i) rot(2,i) rot(3,i)]);
    pause(0.0001)
end
hold off
figure(2)
plot(t,omega(1,:),'r');
hold on
plot(t,omega(2,:),'b');
plot(t,omega(3,:),'k');
plot(t,omega(4,:),'g');
hold off
figure(3)
plot(x,y);
hold on
plot3(x,y,z,'m');
hold off
% quad.EOM
% moment = quad.M

%%
clc
Coriolis
%%
a = ReadProperty("CloverProp.pdf");
a.values
a.keys
%%
syms T phi tta
m = 0.3;
g = 9.81;
dtrans = [freq;0.2;0.3];
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
m=0.2396;
g=9.81;
simTime = 1;
increment = 0.01;
t = 0:increment:simTime;
amp = 0.5;
freq = pi/2;
x = amp*cos(freq*t);
dx = -amp*freq*sin(freq*t);
d2x = -amp*freq^2*cos(freq*t);
y = amp*sin(freq*t);
dy = amp*freq*cos(freq*t);
d2y = -amp*freq^2*sin(freq*t);
z = 0.02*t.^3;
dz = 0.06*t.^2;
d2z = 0.04*t;%ones(1,size(t,2));
% x = zeros(1,numel(t));
% y = zeros(1,numel(t));
% x = -2 + 2*cos( (2*pi/20)*t );
% y = 2*sin( (2*pi/20)*t );
% z = (3/30^2)*t.^2 - (2/30^3)*t.^3;
trans = [x;y;z];
syms thrust phi tta
dtrans = [zeros(3,1), diff(trans,1,2)/increment];
d2trans = [zeros(3,1), diff(trans,2,2)/increment, zeros(3,1)];
dtransdev = [dx;dy;dz];
d2transdev = [d2x;d2y;d2z];
dragMat = diag(0.25*ones(1,3));
rot(3) = 0;
range = [-pi/2 pi/2;-pi/2 pi/2;0 Inf];
% range = [-pi/2 pi/2;-pi/2 pi/2;0 Inf];
eq = [0; 0; thrust] == RotMat([phi tta rot(3)]).'*(m*([0;0;g] + d2transdev(:,10))) % + dragMat*dtrans(:,2)
[phiVal, ttaVal, thrustVal]  = vpasolve(eq, [phi tta thrust], range)
% subs(eq,[phi tta],[phiVal ttaVal])

%%
clc;close all
addpath(genpath('./Addition'));
quadParams = ReadProperty("CloverProp.pdf");
initState = [0; 0; zeros(10,1)];
initInput = zeros(4,1);
simTime = 20;
increment = 0.05;
t = 0:increment:simTime;
quad = Quadcopter(quadParams, initState, initInput, t);
quad.Model3D('CloverAssemblyP.PLY');
axis([-1 1 -1 1 -0.2 3])
camlight
%%
% Trajectory
close all
amp = 0.5;
freq = pi/2;
% x = amp*cos(freq*t);
% dx = -amp*freq*sin(freq*t);
% d2x = -amp*freq^2*cos(freq*t);
% y = amp*sin(freq*t);
% dy = amp*freq*cos(freq*t);
% d2y = -amp*freq^2*sin(freq*t);
% z = 0.002*t.^3;
% dz = 0.006*t.^2;
% d2z = 0.012*t;
a = TrajectoryGeneration(20,3,t);
s = size(a.pos,2);
z = [a.pos, a.pos(end)*ones(1,size(t,2)-s)];
dz = [a.velo, a.velo(end)*ones(1,size(t,2)-s)];
d2z = [a.acce, a.acce(end)*ones(1,size(t,2)-s)];

y = 0*t;
dy = 0*t;
d2y = 0*t;
x = 0*t;
dx = 0*t;
d2x = 0*t;%ones(1,size(t,2));
rot = zeros(3,size(t,2));
drot = zeros(3,size(t,2));
d2rot = zeros(3,size(t,2));
% omega = zeros(4,size(t,2));
% legend
% figure(1)
% hold on
% quad.TrajSim([x;y;z],simTime);
quad.TrajSimulation([x;y;z],[dx;dy;dz],[d2x;d2y;d2z],simTime);
% omega = double(real(quad.omega));
omega = quad.omega;
% plot(t,quad.omega);
% hold off

figure(2)
title('Omega');
hold on
plot(t,omega(1,:),'r');
plot(t,omega(2,:),'b');
plot(t,omega(3,:),'k');
plot(t,omega(4,:),'g');
legend('w1','w2','w3','w4')
hold off

figure(3)
title('Difference');
hold on
plot(t,(x-quad.transCur(1,:)),'r');
plot(t,(y-quad.transCur(2,:)),'g');
plot(t,(z-quad.transCur(3,:)),'b');
legend('x','y','z')
hold off

figure(4)
plot(t,quad.T)
title('Thrust');
figure(5)
plot(t,quad.M)
legend('phi','theta','psi')
title('Torque');
figure(6)
% plot3(quad.trans(1,:),quad.trans(2,:),quad.trans(3,:))
% hold on
% plot3(quad.transForw(1,:),quad.transForw(2,:),quad.transForw(3,:))
% hold off
% plot(x,y);
hold on
plot3(x,y,z,'m');
plot3(quad.transCur(1,:),quad.transCur(2,:),quad.transCur(3,:))
legend('Desired','Real')
title('Traj');
hold off

figure(7)
hold on
subplot(2,3,1);

plot(t,quad.d2rotDes-quad.d2rotCur);
title('Ang Acce');
subplot(2,3,2);

plot(t,quad.drotDes-quad.drotCur);
title('Ang Velo');
subplot(2,3,3);

plot(t,quad.rotDes-quad.rotCur);
title('Ang');
subplot(2,3,4);

plot(t,quad.d2transDes-quad.d2transCur);
title('Trans Acce');
subplot(2,3,5);

plot(t,quad.dtransDes-quad.dtransCur);
title('Trans Velo');
subplot(2,3,6);

plot(t,quad.transDes-quad.transCur);
title('Trans');
legend('x','y','z')

figure(8)
plot(t,quad.rotCur(2,:))
hold on
plot(t,quad.rotCur(1,:))

figure(9)
plot(t,quad.transDes(3,:),t,quad.transCur(3,:))

%%
close all
inc = 0.01;
timeEnd = 20;
t = 0:inc:timeEnd;
a = 2.9807;
b = 0.5;
c = 6;
startDece = find(t==round(4*b+c,2));
[accelerate, endAcce] = f(a,b,1,t);
decelerate = finv(a,b,c,startDece,t);
steady = zeros(1,startDece - endAcce);
jounce = [accelerate steady decelerate];
jerk = cumtrapz(inc,jounce);
acce = cumtrapz(inc,jerk);
velo = cumtrapz(inc,acce);
pos = cumtrapz(inc,velo);
s = size(jounce,2);
tiledlayout(3,3);

nexttile
hold on
plot(t(1:s),jounce)
hold off
xlim([0 timeEnd]);
title('Jounce');

nexttile
plot(t(1:s), jerk);
xlim([0 timeEnd]);
title('Jerk');

nexttile
plot(t(1:s), acce);
xlim([0 timeEnd]);
title('Acce');

nexttile
plot(t(1:s), velo);
xlim([0 timeEnd]);
title('Velo');

nexttile
plot(t(1:s), pos);
xlim([0 timeEnd]);
title('Pos');

% figure(2)
% plot(t(1:200), pos(1:200));
% hold on
% f1 = (a*b^4/pi^4)*sin(pi*t/b) + (a*b/(6*pi))*t.^3 - ((a*b^3)/pi^3)*t;
% plot(t,f1)

figure(2)
a1 = a*sin(pi*t(startDece:startDece+b/inc)/b - (4*b+c)*pi/b);
plot(t(startDece:startDece+b/inc),a1)

%%
close all;
inc = 0.02;
timeEnd = 15;
t = 0:inc:timeEnd;
a = TrajectoryGeneration(5,3,t); %pos>=6, inc = 0.01 (must)
a1 = TrajectoryGeneration(5,1,t);
s = size(a.acce,2);
s1 = size(a1.acce,2);
%%
addpath(genpath('./Support'));
quadParams = ReadProperty("CloverProp.pdf");
initState = [0; 0; 30; zeros(9,1)];
initInput = zeros(4,1);

quad = Quadcopter(quadParams, initState, initInput, t);

%%

% z = [(30-a.pos), (30-a.pos(end))*ones(1,size(t,2)-s)];
% dz = [a.velo, a.velo(end)*ones(1,size(t,2)-s)];
% d2z = [a.acce, a.acce(end)*ones(1,size(t,2)-s)];
z = 30*ones(1,size(t,2));
dz = 0*t;
d2z = 0*t;
% z = [a.pos, a.pos(end)*ones(1,size(t,2)-s)];
% dz = [a.velo, a.velo(end)*ones(1,size(t,2)-s)];
% d2z = [a.acce, a.acce(end)*ones(1,size(t,2)-s)];

x = [a1.pos, a1.pos(end)*ones(1,size(t,2)-s1)];
dx = [a1.velo, a1.velo(end)*ones(1,size(t,2)-s1)];
d2x = [a1.acce, a1.acce(end)*ones(1,size(t,2)-s1)];

y = 0*t;
dy = 0*t;
d2y = 0*t;
% x = 0*t;
% dx = 0*t;
% d2x = 0*t;%ones(1,size(t,2));
rot = zeros(3,size(t,2));
drot = zeros(3,size(t,2));
d2rot = zeros(3,size(t,2));
%%
close all
quad.TrajSimulation([x;y;z],[dx;dy;dz],[d2x;d2y;d2z]);
disp('Done');
PlotTrajGen(a1,t);

figure(2)
plot(t,quad.omega)
xlabel('Time (s)')
ylabel('Velocities (rad/s)')
hLeg(7) = legend('$$\omega_{1}$$','$$\omega_{2}$$','$$\omega_{3}$$','$$\omega_{4}$$');
title('Angular velocities of rotors')

figure(3)
subplot(1,3,1)
plot(t,quad.d2transCur)
subplot(1,3,2)
plot(t,quad.dtransCur)
subplot(1,3,3)
plot(t,quad.transCur)

figure(4)
plot(t,quad.T)
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Thrust of rotors');

figure(5)
plot(t,quad.M)
xlabel('Time (s)')
ylabel('Torque (N)')
hLeg(8) = legend('$$\tau_{\phi}$$','$$\tau_{\theta}$$','$$\tau_{\psi}$$');
title('Torque of rotors');

figure(7);
hold on

subplot(2,3,1);
plot(t,quad.d2rotDes-quad.d2rotCur);
xlabel('Time (s)')
ylabel('Acceleration (rad/s^2)')
hLeg(1) = legend('$$\Delta\ddot{\phi}$$','$$\Delta\ddot{\theta}$$','$$\Delta\ddot{\psi}$$');
title('Angular Acceleration');

subplot(2,3,2);
plot(t,quad.drotDes-quad.drotCur);
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
hLeg(2) = legend('$$\Delta\dot{\phi}$$','$$\Delta\dot{\theta}$$','$$\Delta\dot{\psi}$$');
title('Angular Velocity');

subplot(2,3,3);
plot(t,quad.rotDes-quad.rotCur);
xlabel('Time (s)')
ylabel('Angle (rad)')
hLeg(3) = legend('$$\Delta\phi$$','$$\Delta\theta$$','$$\Delta\psi$$');
title('Angle');

subplot(2,3,4);
plot(t,quad.d2transDes-quad.d2transCur);
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
hLeg(4) = legend('$$\Delta\ddot{x}$$','$$\Delta\ddot{y}$$','$$\Delta\ddot{z}$$');
title('Translational Acceleration');

subplot(2,3,5);
plot(t,quad.dtransDes-quad.dtransCur);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Translational Velocity');
hLeg(5) = legend('$$\Delta\dot{x}$$','$$\Delta\dot{y}$$','$$\Delta\dot{z}$$');

subplot(2,3,6);
plot(t,quad.transDes-quad.transCur);
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Position');
hLeg(6) = legend('$$\Delta{x}$$','$$\Delta{y}$$','$$\Delta{z}$$');
set(hLeg,'Interpreter','latex');

% figure(8)
% plot(t,quad.rotCur(2,:))
% hold on
% plot(t,quad.rotCur(1,:))
% hold off

figure(9)
plot(t,quad.transDes(1,:),t,quad.transCur(1,:))
xlabel('Time (s)')
ylabel('Altitude (m)')
legend('Desired altitude', 'Real altitude')
title('Response of Quadcopter with Altitude control')

%%
figure(6)
fig = tiledlayout(6,6,'TileSpacing','Compact');
title(fig,'Trajectory','FontWeight','bold')
xlabel(fig,'Time (s)')
ax = nexttile(1,[6 5]);
quad.Model3D('CloverAssemblyP.PLY',ax);
axis([-0.5 8 -1 1 -0.1 30])
% axis equal
hold on
% length = 0.2;
% o = -0.09;
% xFloor = [-length,-length;length,length];  yFloor = [-length,length;-length,length];  zFloor = [o, o; o, o];
% PlotEnvironment('LandingPad.jpg', xFloor, yFloor, zFloor)
hLgt = camlight('right');
%%
for i = 1:20:size(t,2)

   nexttile(1,[6 5])
   quad.Animation(i);
   ylabel('Position (m)')
%    zlim([(quad.transCur(3,i)-1) (quad.transCur(3,i)+1)])
%    axis([-0.5 0.5 -0.5 0.5])
   camlight(hLgt,'right')
   pause(0.01)
   
   nexttile(6)
   plot(t(i),quad.transCur(:,i),'.')
   hold on
   ylabel('Position (m)')
   title('Position');

   nexttile(12)
   plot(t(i), quad.dtransCur(:,i),'.');
   hold on
   ylabel('Velo (m/s^2)')
   title('Trans Velo');

   nexttile(18)
   plot(t(i), quad.d2transCur(:,i),'.');
   hold on
   ylabel('Acce (m/s^2)')
   title('Trans Acce');

   nexttile(24)
   plot(t(i), quad.rotCur(:,i),'.');
   hold on
   ylabel('Angle (rad)')
   title('Angle');

   nexttile(30)
   plot(t(i), quad.rotCur(:,i),'.');
   hold on
   ylabel('Velo (rad/s)')
   title('Ang Velo');

   nexttile(36)
   plot(t(i), quad.rotCur(:,i),'.');
   hold on
   ylabel('Acce (rad/s^2)')
   title('Ang Acce');
end
nexttile(1,[6 5])
% axis([-0.5 0.5 -0.5 0.5 -0.2 30])
   
%%
clear;clc;
scene1 = uavScenario(ReferenceLocation=[-33.876 151.19 0]); %-33.876 151.19 %40.707088 -74.012146
% addMesh(scene1,"terrain",{"gmted2010",[-500 500],[-500 500]},[0.5 0.5 0.5]);
% addMesh(scene1,"buildings",{"Copy_of_mapWPark.osm",[-500 500],[-500 500],"auto"},[1 0 0]);
addMesh(scene1,"buildings",{"bridge.osm",[-5500 5500],[-5500 5500],"auto"},[0 1 0]);
show3D(scene1)
%%
t=0:0.04:10;
[h, hEnd] = f(5,0.5,1,t);
plot(t(1:size(h,2)),h);

function [F, endVec] = f(a,b,start,t)
   idx = b/(t(2)-t(1));
   f1 = a*sin(pi*t(start:start+idx)/b);
   f2 = -a*sin(pi*t(start+idx+1:start+3*idx)/(2*b) - pi/2);
   f3 = a*sin(pi*t(start+3*idx+1:start+4*idx-1)/b - 3*pi);
   F = [f1 f2 f3];
   endVec = start+4*idx;
end

function [F, endVec] = finv(a,b,c,start,t)
   idx = b/(t(2)-t(1));
   f1 = a*sin(pi*t(start:start+idx)/b - (4*b+c)*pi/b);
   f2 = -a*sin(pi*t(start+idx+1:start+3*idx)/(2*b) - (5*b+c)*pi/(2*b));
   f3 = a*sin(pi*t(start+3*idx+1:start+4*idx-1)/b - (7*b+c)*pi/b);
   F = -[f1 f2 f3];
   endVec = start+4*idx;
end


