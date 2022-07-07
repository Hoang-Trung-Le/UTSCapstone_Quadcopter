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
drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

drone1_initStates = [0, 0, -5, ...                                              % x, y, z
    0, 0, 0, ...                                                                % dx, dy, dz
    0, 0, 0, ...                                                                % phi, theta, psi
    0, 0, 0]';                                                                  % p, q, r

drone1_initInputs = [0, ...                                                     % ThrottleCMD
    0, 0, 0]';                                                                  % R, P, Y CMD

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
			  
drone1_gains = containers.Map(...
	{'P_phi','I_phi','D_phi',...
	'P_theta','I_theta','D_theta',...
	'P_psi','I_psi','D_psi',...
	'P_zdot','I_zdot','D_zdot'},...
    {0.2, 0.0, 0.15,...
	0.2, 0.0, 0.15,...
	0.8, 0.0, 0.3,...
	10.0, 0.2, 0.0});

simulationTime = 2;
quad = Quadcopter(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains, simulationTime);
quad.Model3D('CloverAssemblyP.PLY');


%%
clc
Coriolis
%%
a = ReadProperty("CloverProp.pdf")
a.values
a.keys


