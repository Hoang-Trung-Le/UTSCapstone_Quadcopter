%% Package Delivery Quadcopter Parameter Initialization

% Copyright 2021 The MathWorks, Inc.
clear;clc;
%% Time Step
Tsc = 1e-3;
Ts = 1e-3;

%% Size of the ground
% infPlane.x = 2;%12.5;              % m
% infPlane.y = 2;%8.5;               % m
% infPlane.z = 2;%0.2;               % m
planeX = 30;
planeY = 30;
planeZ = 0.2;
%% Package size and density
prcSize = [0.1 0.1 0.1];     % m
pkgDensity = 1;                 % kg/m^3

%% Package ground contact properties
pkgGrndStiff  = 10000;          % N/m
pkgGrndDamp   = 30;             % N/(m/s)
pkgGrndTransW = 1e-6;           % m

%% Propeller Characteristics
propD    = 0.12;  % m
propHoverSpeed = 600;    % rpm

%% Initial Position and Orientation of the Quadcopter
%  Position
% xStart = -5;                        % m
% yStart = -3;                        % m
% zStart = 0.06;                      % m

xStart = 0;                        % m
yStart = 0;                        % m
zStart = 0.06;                      % m

%  Orientation (Euler Angles)
xrot = 0;                       % deg
yrot = 0;                       % deg
zrot = 0;                       % deg

%% Material Property
rho_nylon = 1.41;               % g/cm^3 
rho_glass = 2.56;               % g/cm^3
rho_pla   = 1.25;               % g/cm^3 
rho_cfrp  = 6.32/3;             % g/cm^3
rho_al    = 2.66;               % g/cm^3

%% Trajectory Generation
% Waypoints
xWaypt = [0 -1 -2]';
yWaypt = [0 0 0]';
zWaypt = [3 3 1]';
wayPoints = [xWaypt yWaypt zWaypt]';

xTrajPts = [xStart;xWaypt];
yTrajPts = [yStart;yWaypt];
zTrajPts = [zStart;zWaypt];

% Nominal Velocity
V_nom = 2; % m/s

% Time spot for the trajectory design between the waypoints
timeStamp = zeros(length(xWaypt),1);
for i = 1:1:length(xWaypt)
    dx = xTrajPts(i) - xTrajPts(i+1);
    dy = yTrajPts(i) - yTrajPts(i+1);
    dz = zTrajPts(i) - zTrajPts(i+1);
    tx = abs(dx)/V_nom;
    ty = abs(dy)/V_nom;
    tz = abs(dz)/V_nom;
    timeStamp(i,1) = max([tx ty tz]);
end

% Simulation Stop time
T_stop = 25;

% Target location
% targetX = wayPoints(1,end);
% targetY = wayPoints(2,end);
% targetZ = wayPoints(3,end);
target = wayPoints(:,end);

%% Quadcopter Position Controller Gains
KP_position = 0.175; %0.175
KD_position = 0.85;

%% Quadcopter Attitude Controller Gains
kp_attitude = 8.5;
ki_attitude = 5;
kd_attitude = 40;

%% Quadcopter Altitude Controller Gains
KP_altitude = 0.15; %0.15
ki_altitude = 0.0275;
kd_altitude = 0.475;