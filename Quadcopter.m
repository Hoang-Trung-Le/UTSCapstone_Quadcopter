classdef Quadcopter < handle
    %QUADCOPTER This is a class for dynamic calculation and simulation of quadcopter
    %   Dynamic calculation by Euler-Lagrange equation
    %   Import 3D model of quadcopter
    %   Generate trajectory
    %   Simulate quadcopter
    
    % Parameters and kinematic variables
    properties
        g           % gravity
        t           % current time stamp (percentage in total sim time)
        dt          % time interval
        tf          % total simulation time
        
        m           % mass
        l           % arm length of quadcopter
        I           % moment of inertia matrix
        
        q           % q (X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r) state variables previously "x"
        trans       % (X, Y, Z) translation vars
        dtrans      % (dX, dY, dZ) translation velos
        rot         % (phi, theta, psi) rotation vars
        w           % (p, q, r)   rotation velos in body frame
        
        dq          % dq derivatives of joint states previously "dx"
        
        u           % control input (thrust and 3 moments)
        T           % thrust
        M           % moments
    end
    
    % Properties to construct 3D model
    properties
        model       % 3D model of quadcopter from .ply file
        objPose     % Pose of model
        objMesh_h   % Mesh of model
        objVerts    % Vertices of ply model
        objVertexCount % Number of vertices
    end
    
    properties
        phi_des
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_des
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_des
        psi_err
        psi_err_prev
        psi_err_sum
        
        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum
    end
    
    properties
        KP_phi
        KI_phi
        KD_phi
        
        KP_theta
        KI_theta
        KD_theta
        
        KP_psi
        KI_psi
        KD_psi
        
        KP_zdot
        KI_zdot
        KD_zdot
    end
    
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Quadcopter(params, initStates, initControlInputs, simTime) %params, initStates, initControlInputs, gains, simTime
            obj.g = 9.81;       % gravity
            obj.t = 0.0;        % time stamp
            obj.dt = 0.01;      % time interval
            obj.tf = simTime;   % total simulation time
            
            % Set physical parameters of the quadcopter
            % params is a container with 5 pairs
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),             0,            0;...
                                 0, params('Iyy'),            0;...
                                 0,             0, params('Izz')];
            
            % Initial state of quadcopter
            obj.q = initStates;
            obj.trans = obj.q(1:3);
            obj.dtrans = obj.q(4:6);
            obj.rot = obj.q(7:9);
            obj.w = obj.q(10:12);
            
            % Initiate derivatives of state vars
            obj.dq = zeros(12,1);
            
            % Set control input
            obj.u = initControlInputs;
            obj.T = obj.u(1);           % Set thrust
            obj.M = obj.u(2:4);         % Set 3 torques
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Error
            obj.phi_err = 0.0;
            obj.phi_err_prev = 0.0;
            obj.phi_err_sum = 0.0;
            obj.theta_err = 0.0;
            obj.theta_err_prev = 0.0;
            obj.theta_err_sum = 0.0;
            obj.psi_err = 0.0;
            obj.psi_err_prev = 0.0;
            obj.psi_err_sum = 0.0;
            
            obj.zdot_err = 0.0;
            obj.zdot_err_prev = 0.0;
            obj.zdot_err_sum = 0.0;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Find proper gains for the controller.
            %{
            obj.KP_phi=gains('P_phi');
            obj.KI_phi=gains('I_phi');
            obj.KD_phi=gains('D_phi');
            
            obj.KP_theta=gains('P_theta');
            obj.KI_theta=gains('I_theta');
            obj.KD_theta=gains('D_theta');
            
            obj.KP_psi=gains('P_psi');
            obj.KI_psi=gains('I_psi');
            obj.KD_psi=gains('D_psi');
            
            obj.KP_zdot = gains('P_zdot');
            obj.KI_zdot = gains('I_zdot');
            obj.KD_zdot = gains('D_zdot');
            %}
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end
        
        %% RETURNS DRONE STATE
        function state = GetState(obj)
            state = obj.q;
        end
        
        %% EQUATION OF MOTION (EULER-LAGRANGE EQUATION)
        function obj = EOM(obj)
            
            
            
            
        end
        
        
        %%  IMPORT 3D MODEL OF QUADCOPTER
        function Model3D(obj, model)
            obj.model = model;
            [f,v,data] = plyread(model,'tri');
            % Get vertex count
            obj.objVertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v) / obj.objVertexCount;
            obj.objVerts = v - repmat(midPoint, obj.objVertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
            obj.objPose = eye(4);
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
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
        end
        
        
        
        %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
            bRi = RPY2Rot(obj.rot);
            R = bRi';
            
            % Translational Motions
            obj.dq(1:3) = obj.dtrans;
            obj.dq(4:6) = 1 / obj.m * ([0; 0; obj.m * obj.g] + R * obj.T * [0; 0; -1]);
            
            % Rotational Motions
            phi = obj.rot(1); theta = obj.rot(2);
            obj.dq(7:9) = [1    sin(phi)*tan(theta) cos(phi)*tan(theta);
                0    cos(phi)            -sin(phi);
                0    sin(phi)*sec(theta) cos(phi)*sec(theta)] * obj.w;
            
            obj.dq(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
            
        end
        
        %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            obj.EvalEOM();
            obj.q = obj.q + obj.dq.*obj.dt;
            
            obj.trans = obj.q(1:3);
            obj.dtrans = obj.q(4:6);
            obj.rot = obj.q(7:9);
            obj.w = obj.q(10:12);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Simulate the actual sensor output you can measure from the drone.
            % Examples down here are not accurate model
            %             obj.w(1) = obj.w(1) + randn() + 0.2;
            %             obj.w(2) = obj.w(2) + randn() + 0.2;
            %             obj.w(3) = obj.w(3) + randn() + 0.5;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
        %% CONTROLLER
        function obj = AttitudeCtrl(obj, refSig)
            obj.phi_des = refSig(1);
            obj.theta_des = refSig(2);
            obj.psi_des = refSig(3);
            obj.zdot_des = refSig(4);
            
            obj.phi_err = obj.phi_des - obj.rot(1);
            obj.theta_err = obj.theta_des - obj.rot(2);
            obj.psi_err = obj.psi_des - obj.rot(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% QUIZ #0 %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Write a code for the rate controller (PID controller for now)
            % that produces RPY moments. Position of the drone may change
            % since we are dealing with the attitude of the drone only.
            % (e.g.) obj.M(1,1) = ~~~; obj.M(2,1) = ~~~; obj.M(3,1) = ~~~;
            obj.u(2) = (obj.KP_phi * obj.phi_err + ...
                obj.KI_phi * (obj.phi_err_sum) + ...
                obj.KD_phi * (0 - obj.w(1))); % With small angle Approx.
            %(Diff. is not stable in micro_processors such as DSP, ARM)(Gain might differ)
            %                         obj.KD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);
            
            
            
            obj.u(3) = (obj.KP_theta * obj.theta_err + ...
                obj.KI_theta * (obj.theta_err_sum) + ...
                obj.KD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);
            
            obj.u(4) = (obj.KP_psi * obj.psi_err + ...
                obj.KI_psi * (obj.psi_err_sum) + ...
                obj.KD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.phi_err_prev = obj.phi_err;
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err;
            obj.theta_err_prev = obj.theta_err;
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err;
            obj.psi_err_prev = obj.psi_err;
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Basic motor output assumed to make our problem easier.
            % Remove it when you want to put a realistic simulation
            % environment with the actual motor dynamics.
            obj.zdot_err = obj.zdot_des - obj.dtrans(3);
            
            obj.u(1) = obj.m * obj.g;
            obj.u(1) = obj.m * obj.g - ...
                (obj.KP_zdot * obj.zdot_err + ...
                obj.KI_zdot * (obj.zdot_err_sum) + ...
                obj.KD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);
            
            obj.zdot_err_prev = obj.zdot_err;
            obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %             obj.u(1) = 2*obj.m * obj.g;
            %             obj.u(2) = 0.0;
            %             obj.u(3) = 0.0;
            %             obj.u(4) = 0.0;
            
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
        end
    end
    
    methods
        
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
    end
    
end

