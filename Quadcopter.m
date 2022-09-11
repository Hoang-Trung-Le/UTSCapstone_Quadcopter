classdef Quadcopter < handle
   %QUADCOPTER This is a class for dynamic calculation and simulation of quadcopter
   %   Dynamic calculation by Euler-Lagrange equation
   %   Import 3D model of quadcopter
   %   Generate trajectory
   %   Simulate quadcopter


   properties (Constant)
      g = [0; 0; 9.81]                 % Gravity
      dragMat = diag(0.25*ones(1,3));  % Drag coefficients
      b = 1.14*10^-7                   % Drag constant
      k = 2.98*10^-6                   % Lift constant
   end


   % Parameters and kinematic variables
   properties
      t           % current time stamp (percentage in total sim time)
      dt          % time interval
      tf          % total simulation time
      simSize     % size of simulation trajectory

      m          % mass
      l          % arm length of quadcopter
      I          % moment of inertia matrix

      q           % q (X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r)
      transDes       % (X, Y, Z) desired translation vars
      dtransDes      % (dX, dY, dZ) desired translation velos
      d2transDes     % (d2X, d2Y, d2Z) desired translation acces
      rotDes         % (phi, theta, psi) desired rotation vars
      drotDes        % (dphi, dtheta, dpsi) desired rotation velos
      d2rotDes       % (d2phi, d2theta, d2psi) desired rotation acces
%       w           % (p, q, r)   rotation velos in body frame

%       dq          % dq derivatives of joint states

      u           % control input (thrust and 3 moments)
      T           % thrust
      M           % moments

      omega       % Angular velocities of rotors

      transCur       % (X, Y, Z) real translation vars
      dtransCur      % (dX, dY, dZ) real translation velos
      d2transCur     % (d2X, d2Y, d2Z) real translation acces
      rotCur         % (phi, theta, psi) real rotation vars
      drotCur        % (dphi, dtheta, dpsi) real rotation velos
      d2rotCur       % (d2phi, d2theta, d2psi) real rotation acces

   end

   properties (GetAccess = public, SetAccess = private)
      aw = 7;
   end


   % Properties to construct 3D model
   properties (Access = private)
      model       % 3D model of quadcopter from .ply file
      objPose     % Pose of model
      objMesh_h   % Mesh of model
      objVerts    % Vertices of ply model
      objVertexCount % Number of vertices
   end

   properties

      transErr
      dtransErr
      d2transErr
      d2transCtrl

      rotErr
      drotErr

   end

   %% PID GAINS
   properties

      KPtrans
      KDtrans
      KDDtrans
      zErrSum
      KItrans

      KProt
      KDrot

   end

   %% METHODS
   methods
      %% CONSTRUCTOR
      function obj = Quadcopter(params, initStates, initControlInputs, simTime)
         obj.t = 0.0;                           % time stamp
         obj.dt = simTime(2) - simTime(1);      % time interval
         obj.tf = simTime(end);                 % total simulation time

         % Set physical parameters of the quadcopter
         % params is a container with 5 pairs
         obj.m = params('mass');
         obj.l = params('armLength');
         obj.I = diag([params('Ixx'), params('Iyy'), params('Izz')]);

         obj.simSize = size(simTime,2);

         % Initial state of quadcopter
         obj.q = initStates;
         obj.rotCur = [obj.q(7:9), zeros(3,obj.simSize-1)];
         obj.drotCur = [obj.q(10:12), zeros(3,obj.simSize-1)];
         obj.d2rotCur = zeros(3,obj.simSize);
         obj.transCur = [obj.q(1:3), zeros(3,obj.simSize-1)];
         obj.dtransCur =  [obj.q(4:6), zeros(3,obj.simSize-1)];
         obj.d2transCur =  zeros(3,obj.simSize);
         
         obj.transDes = zeros(3,obj.simSize);
         obj.dtransDes = zeros(3,obj.simSize);
         obj.d2transDes = zeros(3,obj.simSize);
         obj.rotDes = zeros(3,obj.simSize);
         obj.drotDes = zeros(3,obj.simSize);
         obj.d2rotDes = zeros(3,obj.simSize);
%          obj.w = obj.q(10:12);

         % Initialise derivatives of state vars
%          obj.dq = zeros(12,1);

         % Set control input
         obj.u = initControlInputs;
         obj.T = [obj.u(1), zeros(1,obj.simSize-1)];           % Set thrust
         obj.M = [obj.u(2:4), zeros(3,obj.simSize-1)];        % Set 3 moments

         % Set angular velocities of rotors
         obj.omega = zeros(4,obj.simSize);

         % Initialise Error
         obj.transErr = zeros(3,obj.simSize);
         obj.dtransErr = zeros(3,obj.simSize);
         obj.d2transErr = zeros(3,obj.simSize);
         obj.d2transCtrl = zeros(3,obj.simSize);

         obj.rotErr = zeros(3,obj.simSize);
         obj.drotErr = zeros(3,obj.simSize);

      end

      %% TRAJECTORY SIMULATION
      function TrajSimulation(obj, traj, dtraj, d2traj)

         obj.KPtrans = [2; 2; 2];
         obj.KDtrans = [0.75;0.75;0.75];
         obj.KDDtrans = [1;1;1];

         obj.KItrans = [0;0;0];
         obj.zErrSum = zeros(3,obj.simSize);

         obj.KProt = [3;3;3];
         obj.KDrot = [0.75;0.75;0.75];

         obj.zErrSum = 0;

         obj.transDes = traj;
         obj.dtransDes = dtraj;
         obj.d2transDes = d2traj;

         for i = 2:obj.simSize

            obj.PositionControl(i);
            obj.PostureControl(i);
            obj.TransAcce(i);
            obj.TransEstimator(i);
            obj.drotDes(:,i) = obj.ApproxDev(obj.rotDes,i);
            obj.AttitudeControl(i);
            obj.AngAcce(i);
            obj.AngEstimator(i);
            obj.ControlInputs(i);

         end
      end


      %% APPROXIMATION of DERIVATIVE
      function a = ApproxDev(obj, att, i)
         dev = gradient(att(1:i), obj.dt);
         a = dev(end);
      end

      %% TRAJECTORY CONTROL (EULER-LAGRANGE EQUATION) (LINEAR COMPONENT)
      function PostureControl(obj,i)
         syms thrust phi tta
         obj.rotDes(3,i) = 0;
         range = [-pi/2 pi/2; -pi/2 pi/2; -Inf Inf];
         eq = [0; 0; thrust] == RotMat([phi tta obj.rotDes(3)]).'*(obj.m*(obj.g + obj.d2transCtrl(:,i)) + obj.dragMat*obj.dtransCur(:,i-1));
         [obj.rotDes(1,i), obj.rotDes(2,i), obj.T(i)]  = vpasolve(eq, [phi, tta, thrust], range);
      end


      %% EQUATION OF MOTION (EULER-LAGRANGE EQUATION) (ANGULAR COMPONENT)
      function MotionEq(obj, start)
         for i = start:obj.simSize
            obj.M(:,i) = Ja(obj.I, obj.rotDes(1,i), obj.rotDes(2,i))*obj.d2rotDes(:,i) + ...
               Coriolis(obj.I, obj.rotDes(:,i), obj.drotDes(:,i))*obj.drotDes(:,i);
         end
      end


      %% CONTROL INPUT CALCULATION
      function ControlInputs(obj, i)
         obj.omega(:,i) = (sqrt(obj.T(i)/(4*obj.k) + [-1 0 -1; 1 1 0; -1 0 1; 1 -1 0] * ...
            [(obj.M(3,i)/(4*obj.b)); (obj.M(1,i)/(2*obj.l*obj.k)); (obj.M(2,i)/(2*obj.l*obj.k))]));
      end

      %% REAL ANGULAR and TRANSLATIONAL ACCELERATIONS CALCULATION
      function AngAcce(obj,i)
         obj.d2rotCur(:,i) = Ja(obj.I, obj.rotCur(1,i-1), obj.rotCur(2,i-1)) \ ...
            (obj.M(:,i) - Coriolis(obj.I, obj.rotCur(:,i-1), obj.drotCur(:,i-1))*obj.drotCur(:,i-1));
         obj.d2rotCur(3,i) = 0;
      end

      function TransAcce(obj,i)
         obj.d2transCur(:,i) = (RotMat(obj.rotCur(:,i-1))*[0;0;obj.T(i)] - obj.dragMat*obj.dtransCur(:,i-1))/obj.m - obj.g;
      end


      %% STATE ESTIMATOR
      function AngEstimator(obj,i)
         obj.drotCur(:,i) = obj.drotCur(:,i-1) + obj.d2rotCur(:,i-1)*obj.dt;
         obj.rotCur(:,i) = obj.rotCur(:,i-1) + obj.drotCur(:,i-1)*obj.dt;
      end

      function TransEstimator(obj,i)
         obj.dtransCur(:,i) = obj.dtransCur(:,i-1) + obj.d2transCur(:,i-1)*obj.dt;
         obj.transCur(:,i) = obj.transCur(:,i-1) + obj.dtransCur(:,i-1)*obj.dt;
      end


      %% CONTROLLER
      function PositionControl(obj,i)
         obj.transErr(:,i) = obj.transDes(:,i) - obj.transCur(:,i-1);
         obj.dtransErr(:,i) = obj.dtransDes(:,i) - obj.dtransCur(:,i-1);
         obj.d2transErr(:,i) = obj.d2transDes(:,i) - obj.d2transCur(:,i-1);

         obj.d2transCtrl(:,i) = obj.KPtrans.*obj.transErr(:,i) + ...
                                obj.KDtrans.*obj.dtransErr(:,i) + ...
                                obj.KDDtrans.*obj.d2transErr(:,i) + ...
                                obj.KItrans.*obj.zErrSum;
         obj.zErrSum = obj.zErrSum + obj.transErr(:,i);
      end

      %%
      function AttitudeControl(obj,i)
         obj.rotErr(:,i) = obj.rotDes(:,i) - obj.rotCur(:,i-1);
         obj.drotErr(:,i) = obj.drotDes(:,i) - obj.drotCur(:,i-1);

         obj.M(:,i) = (obj.KProt.*obj.rotErr(:,i) + obj.KDrot.*obj.drotErr(:,i)) .* diag(obj.I);
      end
   end


   %% CAD MODEL of QUADCOPTER
   methods (Access = public)
      %%  IMPORT 3D MODEL OF QUADCOPTER
      function Model3D(obj, model)
         obj.model = model;
         [f,v,data] = plyread(model,'tri');
         % Get vertex count
         obj.objVertexCount = size(v,1);
         % Move center point to origin
         midPoint = sum(v) / obj.objVertexCount;
         obj.objVerts = v - repmat(midPoint, obj.objVertexCount,1);
         % Create a transform to describe the location (at the origin, since it's centered)
         obj.objPose = eye(4);
         % Scale the colours to be 0-to-1 (they are originally 0-to-255)
         try
            try
               vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            catch
               try
                  vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;
               catch
                  vertexColours = [0.5,0.5,0.5];
               end
            end
            obj.objMesh_h = trisurf(f, obj.objVerts(:,1), obj.objVerts(:,2), obj.objVerts(:,3) ...
               ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
         catch ME_1
            disp(ME_1);
         end
         obj.MoveObj([obj.transDes(:,1); obj.rotDes(:,1)]);
      end

      %%  MOVE 3D MODEL OF QUADCOPTER
      function MoveObj(obj, pose)
         obj.objPose = eye(4);
         % Move forwards (facing in -y direction)
         forwardTR = makehgtform('translate',pose(1:3));
         % Random rotate about Z
         zRotateTR = makehgtform('zrotate',pose(6));
         yRotateTR = makehgtform('yrotate',pose(5));
         xRotateTR = makehgtform('xrotate',pose(4));
         % Move the pose forward and a slight and random rotation
         obj.objPose = obj.objPose * forwardTR * xRotateTR * yRotateTR * zRotateTR;
         updatedPoints = (obj.objPose * [obj.objVerts, ones(obj.objVertexCount,1)]')';
         % Now update the Vertices
         obj.objMesh_h.Vertices = updatedPoints(:,1:3);
      end

      %% QUADCOPTER ANIMATION
      function Animation(obj,i)
         obj.MoveObj([obj.transCur(:,i); obj.rotCur(:,i)]);
         drawnow;
      end

   end
end


