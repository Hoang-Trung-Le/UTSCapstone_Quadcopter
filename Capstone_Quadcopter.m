%% DRONE CONTROL for AUTONOMOUS DELIVERY SYSTEM
% *UNIVERSITY OF TECHNOLOGY SYDNEY*
% *FACULTY OF ENGINEERING AND INFORMATION TECHNOLOGY*
% Student name (Author): LE, Hoang Trung
% Student ID number: 13993807
% Project number: AUT-22-07547
% Major: Mechatronic Engineering

%% INITIALISATION
... struggling
clear;    clc;
addpath(genpath('./Addition'));     % Include required files in separate folder

quadParams = ReadProperty("CloverProp.pdf");
initState = zeros(12,1);
initInput = zeros(4,1);
simTime = 20;

%% INITIALISATION

quad = Quadcopter(quadParams, initState, initInput, simTime);
quad.Model3D('CloverAssemblyP.PLY');

%% TRAJECTORY



%%
% Translational
% Linear translation force
syms F1 F2 F3 F4 thrust
% thrust = F1 + F2 + F3 + F4;
droneThrust = [0; 0; thrust];    % thrust

% Translational variables
syms x y z xD yD zD x2D y2D z2D
trlD = [xD; yD; zD];
trl2D = [x2D; y2D; z2D];

% Rotational
% Euler angle
syms phi tta psi phiD ttaD psiD phi2D tta2D psi2D
rot = [phi; tta; psi];
rotD = [phiD; ttaD; psiD];
rot2D = [phi2D; tta2D; psi2D];

% Angular velocity in body frame
syms omg_phi omg_tta omg_psi omg_r1 omg_r2 omg_r3 omg_r4
omg = [omg_phi omg_tta omg_psi];      % angular velocity of quadcopter
omg_r = -omg_r1+omg_r2-omg_r3+omg_r4; % residual angular velocity corresponding angular velocities of the propellers

% Torque
syms t1 t2 t3 t4 l torque tau_psi
l = 0.12;
tau_phi = (F2-F4)*l;    % l is arm length of quadcopter
tau_tta = (F1-F3)*l;
% tau_psi = t1 + t2 + t3 + t4;
tau = [tau_phi; tau_tta; tau_psi];

% Generalised coordinate in vector q
q = [x y z phi tta psi];
qd = [trlD; rotD];
q2d = [x2D y2D z2D phi2D tta2D psi2D];

% Constant
syms m g k_ax k_ay k_az;
g = 9.81;
grav = [0; 0; g];
m = 0.2275;
Fn_g = [0;0;m*g];
xDragCoef = 0.25;
yDragCoef = 0.25;
zDragCoef = 0.25;
airDragCoef = [xDragCoef yDragCoef zDragCoef]; % aerodynamic friction factors

syms Ixx Iyy Izz Ir       % Ir is moment of inertia of motor's rotor
Ixx = 0.0025;
Iyy = 0.0025;
Izz = 0.0045;
I = [Ixx     0     0;
       0   Iyy     0;
       0     0   Izz;];

omgMotor = sym('omg', [1 4]);
thrustCoef = 2.98*10^-6;
torqueCoef = 1.14*10^-7;

%% Formulation

R = Rz(psi) * Ry(tta) * Rx(phi);     % Rotation matrix (from body to origin)

% Matrix describing the relation between angular velocity and rotation rate
w = [1          0           -sin(tta)
     0   cos(phi)   sin(phi)*cos(tta)
     0  -sin(phi)   cos(phi)*cos(tta)];   

Pe = m*g*z;    % Potential energy

% Use the simplified version of kinematic energy (KE) to avoid computational nightmare
Ke = 0.5*((trlD.')*m*trlD + rotD.'*(w.')*I*w*rotD);   % a more completed version of KE

% Ke = 0.5*(trlD.'*m*trlD + rotD.'*I*rotD);    % angular velocity approximates rotation rate
% To avoid conjugate in transpose symbolic matrix, consider using A.' instead of A'
La = Ke - Pe;   % Initial Lagrangian

Lq = transpose(jacobian(La,q));

Lqd = transpose(subs(jacobian(La,qd),transpose(qd),q2d));

trl_ext = R*droneThrust;    % External translational force

% Gyroscopic torque due to rotation of quadcopter's symmetric body
tau_gyb = -sk(omg_phi, omg_tta, omg_psi) * I * transpose(omg);
%Gyroscopic torque due to rotation of propellers
tau_gyp = [Ir*omg_r*omg_phi; Ir*omg_r*omg_tta; 0];
% Air drag torque
tau_air = transpose(airDragCoef.*omg.^2);
% Total external torque
rot_ext = tau + tau_gyb + tau_gyp - tau_air;

F_ext = [trl_ext; tau];

EL = Lqd - Lq == F_ext;     % Euler-Lagrange equation
% disp(EL)

%%
C = sym(zeros(3));
C(1,2) = (Iyy-Izz)*(ttaD*cos(phi)*sin(phi)+psiD*sin(phi)^2*cos(tta))+(Izz-Iyy)*(psiD*cos(phi)^2*cos(tta))-Ixx*psiD*cos(tta);
C(1,3) = (Izz-Iyy)*psiD*cos(phi)*sin(phi)*cos(tta)^2;
C(2,1) = (Izz-Iyy)*(ttaD*cos(phi)*sin(phi)+psiD*sin(phi)*cos(tta)) + (Iyy-Izz)*psiD*cos(phi)^2*cos(tta) + Ixx*psiD*cos(tta);
C(2,2) = (Izz-Iyy)*phiD*cos(phi)*sin(phi);
C(2,3) = -Ixx*psiD*sin(tta)*cos(tta) + Iyy*psiD*sin(phi)^2*sin(tta)*cos(tta) + Izz*psiD*cos(phi)^2*sin(tta)*cos(tta);
C(3,1) = (Iyy-Izz)*psiD*cos(tta)^2*sin(phi)*cos(phi) - Ixx*ttaD*cos(tta);
C(3,2) = (Izz-Iyy)*(ttaD*cos(phi)*sin(phi)*sin(tta)+phiD*sin(phi)^2*cos(tta)) + (Iyy-Izz)*phiD*cos(phi)^2*cos(tta) + Ixx*psiD*sin(tta)*cos(tta) - Iyy*psiD*sin(phi)^2*sin(tta)*cos(tta) - Izz*psiD*cos(phi)^2*sin(tta)*cos(tta);
C(3,3) = (Iyy-Izz)*phiD*cos(phi)*sin(phi)*cos(tta)^2 - Iyy*ttaD*sin(phi)^2*cos(tta)*sin(tta) - Izz*ttaD*cos(phi)^2*cos(tta)*sin(tta) + Ixx*ttaD*cos(tta)*sin(tta);
% C;
J = (w.')*I*w;
manual = tau == J*rot2D+C*rotD;
%% Trajectory
simuTime = 2;
wayPoints = 50;
timeStamp = linspace(0,simuTime,wayPoints);
%waypointTrajectory

% Position
zStart = 0;
zEnd = 2;
x = 2*cos(timeStamp/simuTime);%zeros(1,wayPoints);%1*timeStamp/simuTime;%
y = zeros(1,wayPoints);%1*timeStamp/simuTime;%
z = (zEnd - zStart)*timeStamp/simuTime;
% phi = zeros(1,100);
% tta = zeros(1,100);
psiNum = zeros(1,wayPoints);%pi*timeStamp/simuTime;

% Velocity
xD = zeros(1, wayPoints);
yD = zeros(1, wayPoints);
zD = zeros(1, wayPoints);
phiD = zeros(1,wayPoints);
ttaD = zeros(1,wayPoints);
psiDNum = zeros(1,wayPoints);
for i = 2:wayPoints
    xD(i) = x(i) - x(i-1);
    yD(i) = y(i) - y(i-1);
    zD(i) = z(i) - z(i-1);
end

% Acceleration
x2D = zeros(1, wayPoints);
y2D = zeros(1, wayPoints);
z2D = zeros(1, wayPoints);
%{
phi2D = zeros(1,wayPoints);
tta2D = zeros(1,wayPoints);
%}
psi2DNum = zeros(1,wayPoints);
for i = 2:wayPoints
    x2D(i) = xD(i) - xD(i-1);
    y2D(i) = yD(i) - yD(i-1);
    z2D(i) = zD(i) - zD(i-1);
end

%%

range = [-pi/2 pi/2;-pi/2 pi/2;-pi pi];
trlDNum = subs(trlD);
trl2DNum = subs(trl2D);

for i = 1:wayPoints
    eqn(:,:,i) = subs(R, psi, psiNum(i))*droneThrust == m*(trl2DNum(:,i) + grav) + diag(airDragCoef)*trlDNum(:,i);
    [phiNum(i), ttaNum(i), thrustNum(i)] = vpasolve(eqn(:,:,i), [phi tta thrust], range);
%     disp(i);
end
phiNum = double(phiNum);    ttaNum = double(ttaNum);    thrustNum = double(thrustNum);
for i = 2:wayPoints
    phiDNum(i) = phiNum(i) - phiNum(i-1);
    ttaDNum(i) = ttaNum(i) - ttaNum(i-1);
    phi2DNum(i) = phiDNum(i) - phiDNum(i-1);
    tta2DNum(i) = ttaDNum(i) - ttaDNum(i-1);
end

%%

for i = 1:4
    torques(i) = torqueCoef * omgMotor(i).^2;
    thrusts(i) = thrustCoef * omgMotor(i).^2;
end
tau_psi = torques(1)-torques(2)+torques(3)-torques(4);
thrustEqn = thrust == sum(thrusts);
rotLag1 = subs(EL(4:6), tau_psi);
rotLag = subs(rotLag1, [F1 F2 F3 F4], [thrusts(1) thrusts(2) thrusts(3) thrusts(4)])
for i = 1:wayPoints
    dynamicSolver(:,:,i) = [subs(rotLag,[rot.' rotD.' rot2D.'], [phiNum(i) ttaNum(i) psiNum(i) phiDNum(i) ttaDNum(i) psiDNum(i) phi2DNum(i) tta2DNum(i) psi2DNum(i)]); subs(thrustEqn, thrust, thrustNum(i))];
    S = vpasolve(dynamicSolver(:,:,i),omgMotor, [500 500 500 500]);

end
manual1 = subs(manual,tau_psi);
manual2 = subs(manual1, [F1 F2 F3 F4], [thrusts(1) thrusts(2) thrusts(3) thrusts(4)]);
for i = 1:wayPoints
    dynamicSolver1(:,:,i) = [subs(manual2,[rot.' rotD.' rot2D.'], [phiNum(i) ttaNum(i) psiNum(i) phiDNum(i) ttaDNum(i) psiDNum(i) phi2DNum(i) tta2DNum(i) psi2DNum(i)]); subs(thrustEqn, thrust, thrustNum(i))];
    [omg1Num(i), omg2Num(i), omg3Num(i), omg4Num(i)] = vpasolve(dynamicSolver1(:,:,i),omgMotor, [0 500; -500 0;0 500; -500 0]);
end

%% Solve for numerical values of dynamic variables
% [ax, bx] = equationsToMatrix(EL == 0, q2d)  % Transform Lagrangian into AX=b form
% q2d_result = linsolve(ax, bx);
% disp(q2d_result)

%% Animation
% figure(1)
% % axis equal
% % axis([-500 1000 -500 500 -200 8000]);
% view(3)
% %%
% % v = VideoWriter('CloverAnimation.avi');
% % open(v);
% Clover = PlaceObj('Clover2.PLY');
% axis equal
% axis([-500 2500 -500 500 -200 3000]);
% %%
% for i = 1:wayPoints
% %     hold on
%     Clover.MoveObj(1000*[x(i) y(i) z(i) phiNum(i) ttaNum(i) psiNum(i)]);
%     pause(0.1);
% frame = getframe(gcf);
% writeVideo(v,frame);
% end
% close(v);
%% Functions

% Rotation matrix
function x = Rx(ang)
    x = [1         0            0;
         0  cos(ang)    -sin(ang);
         0  sin(ang)     cos(ang);];
end

function y = Ry(ang)
    y = [cos(ang)   0   sin(ang);
                0   1          0;
         -sin(ang)  0   cos(ang);];
end

function z = Rz(ang)
    z = [cos(ang)   -sin(ang)   0;
         sin(ang)    cos(ang)   0;
                0           0   1;];
end

% Skew matrix
function Sk = sk(a,b,c)
    Sk = [ 0  -c   b
           c   0  -a
          -b   a   0];
end
