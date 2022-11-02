%%
addpath(genpath('./Support'));
crane(1) = PlaceObj('crane.PLY');
hold on
crane(2) = PlaceObj('crane.PLY');
crane(3) = PlaceObj('crane.PLY');
crane(4) = PlaceObj('crane.PLY');

crane(1).MoveObj([40 48 0 0 0 pi]);
crane(2).MoveObj([74 43 0 0 0 -pi/6]);
crane(3).MoveObj([20 70 0 0 0 pi/2]);
crane(4).MoveObj([70 60 0 0 0 0]);

building = PlaceObj('building.PLY');
building.MoveObj([20 10 0 pi/2 0 0]);
PlotEnvironment('Ground.jpg',[0 100; 0 100], [0 0; 100 100], [0 0; 0 0]);

axis equal
axis([0 100 0 100]);
camlight
