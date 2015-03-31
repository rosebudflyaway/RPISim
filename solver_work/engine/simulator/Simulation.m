%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Simulation.m
%
% Simulates a user-defined 3-d scene.
classdef Simulation < handle
   properties
      P                   % The set of body objects (spheres, cylinders, etc.).
      h                   % The time step.  
      TIME                % Tracks current simulation time.
      STEP                % Step counter
      MAX_ITERS           % Maximum number of iterations before halting
      STATIC_BODY         % The number of static number of bodies
      
      FORMULATION         % What formulation to use (LCP, mLCP, PEG ...)
      SOLVER              % Which solver to use.  Currently, default is PATH
      num_fricdirs        % The number of friction directions in the discretized friction cone
      FRICTION            % Boolean, whether friction is on or off
      
      % Function handles
      hCollisionDetection % Which collision detection function to use (probably never set by user)
      hFormulation        % Which formulation to use e.g. LCP, MCP, NCP. 
      hSolver             % Which solver to use e.g. PATH, Lemke, fixed-point, etc. (Replaces 'SOLVER')
      
      % User interaction variables
      GUI                 % Struct containing user-interface information
      timers              % A struct for monitoring tic/toc times

      RECORD_DATA         % Boolean whether to write simulation data to file.  
      RECORD_CONTACTS     % Boolean whether to write contact data to file.  
      DATA_DIRECTORY      % File to write data to.  
      DATA_fileID         % File id of file to write data to. 
      COLLECT_DATA        % whether to collect the data : bodies contacts constraints
      matFile
      % Contact variables
      Contacts            % Vector of contact objects
      num_Contacts
      num_subContacts     % The total number of contacts including length(C)
      contactID
      bodyID 
      activeBodies        % A vector of bodies in contact, as indicies of P. 
      
      % Joint variables
      Joints              % The set of body joints
      num_jointConstraints
      num_activeJointBodies
      
      FrameCounter          % The frame counter to get rid of the problems without contact
      
      % Dynamics structs
      dynamics            % A struct for holding dynamics information i.e. dynamics.M, dynamics.Gn, dynamics.Gf, etc.
      solution            % A struct for holding solution information i.e. solution.err, solution.iters, etc.
      z                   % Vector where the solution is stored after dynamics solve
      bodies 
      contacts
      constraints
      %con                 % A struct for holding contact information from every step, i.e. con.normal, con.body1, etc.
      data_set
      warmstart
   end
   
   methods
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% Constructor
      function obj = Simulation(P,h)
          obj.P = P;
          for i=1:length(P)
             P{i}.bodyID = i;  
          end
          obj.STATIC_BODY = 0;
          obj.Joints = []; 
          obj.h = h;
          obj.TIME = 0;
          obj.STEP = 0;
          obj.MAX_ITERS = 10^5; 

          obj.solution.tolerance = 0;  % If this is non-zero (and > 0), the solver will use this value for convergence
          obj.FORMULATION = 'LCP'; % Default formluation & solver
          obj.SOLVER = 'Lemke';  
          obj.hFormulation = @LCPdynamics;
          obj.hSolver = @wrap_Lemke;
          obj.num_fricdirs = 6; 
          obj.FRICTION = true; 
          obj.num_jointConstraints = 0;
          obj.num_activeJointBodies = 0; 
 
          obj.data_set = [];
          % Use PATH's courtesy license if one isn't set already.
          if isempty(getenv('PATH_LICENSE_STRING'))
            setenv('PATH_LICENSE_STRING','2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0');
          end
          obj.RECORD_DATA = false; 
          obj.DATA_DIRECTORY = ['sim_data_' datestr(now,'yy_mm_dd_HH_MM_SS')]; 
          obj.matFile = strcat(obj.DATA_DIRECTORY, '/data_set.mat');
          
          obj.GUI.DRAW = true;
          obj.timers.collision_detection = 0;
          obj.timers.solver = 0;
          obj.COLLECT_DATA = 0;
          obj.warmstart = 'quad_program';
          obj.FrameCounter = 1;
          
          % Unfortunately necessary for now...
          for i=1:length(obj.P)
            if strcmp(obj.P{i}.body_type, 'mesh')
              obj.P{i}.update_world_position;
              obj.P{i}.scale(1);
            end
          end
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% Time step
      function obj = step(obj)
        obj.TIME = obj.TIME + obj.h;
        obj.STEP = obj.STEP + 1;
        obj.clearConstraintData(); 
        tic;
        feval(obj.hCollisionDetection, obj);  % Collision Detection
        obj.timers.collision_detection = toc; 
        
        if ~isempty(obj.Contacts) || ~isempty(obj.Joints)
           tic;
             if ~strcmp(obj.FORMULATION, 'particleLCP')
                 preDynamics(obj);                     
             end
              feval(obj.hFormulation, obj);      % Formulate dynamics
             if ~strcmp(obj.FORMULATION, 'particleLCP')
                  obj.timers.formulation = toc;
                  tic;
                 obj.solution = feval(obj.hSolver, obj);           % Solve dynamics
                 obj.timers.solver = toc;
                 obj.z = obj.solution.z;
             end
        end
        kinematic_update(obj); 	                % Apply results of dynamics solution
        postDynamics(obj);                      % Post-solve, post-kinematic update
        if obj.RECORD_DATA, sim_Record(obj); end % Record data for playback
        if obj.RECORD_CONTACTS, sim_RecordContact(obj); end % Record contact info.  
        if obj.COLLECT_DATA && ~isempty(obj.Contacts), sim_CollectData(obj); obj.FrameCounter = obj.FrameCounter + 1; end        
        % Note: sim_RecordContact stores the contact info in obj.con.  In
        % order to save the information to file, sim_RecordContactSave()
        % must be called before the simulator exits.  
        
      end % End step()
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% clearConstraintData()
      function obj = clearConstraintData(obj)
        % Clears contact data
        obj.Contacts = [];         
        obj.num_Contacts = 0;
        obj.num_subContacts = 0;
        obj.activeBodies = [];
        obj.contactID = 1;
        obj.bodyID = 0;  
        for b=1:max(size(obj.P)) 
           obj.P{b}.BodyIndex = 0;    
           obj.P{b}.ContactCount = 0;
        end
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addContact()
      % Adds a contact to the Contacts vector
      function obj = addContact(obj, b1ID,b2ID,n,t,p1,p2,psi_n)
        obj.addActiveBody(b1ID);
        obj.addActiveBody(b2ID); 
        obj.Contacts = [obj.Contacts; contact(obj.contactID, b1ID, b2ID, n, t, p1, p2, psi_n)];
        obj.contactID = obj.contactID + 1;
        obj.num_Contacts = obj.num_Contacts + 1; 
        obj.num_subContacts = obj.num_subContacts + size(psi_n,2);  
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addActiveBody(bid);
      function obj = addActiveBody(obj, bid)
         if obj.P{bid}.static == 0
            obj.P{bid}.ContactCount = obj.P{bid}.ContactCount + 1; 
            if obj.P{bid}.BodyIndex < 1
               obj.bodyID = obj.bodyID+1;
               obj.P{bid}.BodyIndex = obj.bodyID;  
               obj.activeBodies = [obj.activeBodies; bid];
            end   
         end
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addJoint()
      % Input:  body1   The first body
      %         body2   The second body
      %         pos     The world coordinate of the joint
      %         Zaxis   The joint's Z axis in the world frame
      
      % TODO: add joint type
      % TODO: allow body2 to be a point 
      function obj = addJoint(obj, body1, body2, pos, Zaxis, type)
          
          q = [ 1+dot([0;0;1],Zaxis) cross([0;0;1], Zaxis)' ];
          q = q/norm(q);                                        % Rotation to joint frame from world frame

          pos1 = quat2rot(quatinv(body1.quat)) * (pos-body1.u); % Joint position calculated by body1 (world frame)
          pos2 = quat2rot(quatinv(body2.quat)) * (pos-body2.u); % Joint position calculated by body2 (world frame)
          quat1 = quatmultiply(q, quatinv(body1.quat));         % Joint rotation calculated by body1 (world frame)
          quat2 = quatmultiply(q, quatinv(body2.quat));         % Joint rotation calculated by body2 (world frame)
          obj.Joints = [obj.Joints; Joint(type, body1, pos1, quat1, body2, pos2, quat2, obj.num_jointConstraints)];
          obj.Joints(end).jointID = length(obj.Joints); 
          obj.Joints(end).update(); 

          obj.num_jointConstraints = obj.num_jointConstraints + 5;  % TODO: This may not always be 5
          %obj.num_fricdirs = obj.num_fricdirs + 1; What on earth was this about?? 
          
          if ~body1.static && body1.numJoints == 0
              obj.num_activeJointBodies = obj.num_activeJointBodies + 1; 
          end
          if ~body2.static && body2.numJoints == 0
             obj.num_activeJointBodies = obj.num_activeJointBodies + 1; 
          end
          
          body1.doNotCollideWith(body2.bodyID);
          body2.doNotCollideWith(body1.bodyID); 
          body1.numJoints = body1.numJoints + 1;
          body2.numJoints = body2.numJoints + 1;
          
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% formulation()
      % Takes a pre-defined string representing which formulation to use
      function obj = formulation(obj, formulation)
         obj.FORMULATION = formulation; 
         
        % Formulation handle
        if     strcmp(formulation,'LCP'),          obj.hFormulation = @LCPdynamics;
        elseif strcmp(formulation,'mLCP'),         obj.hFormulation = @mLCPdynamics;
        elseif strcmp(formulation,'ST'),           obj.hFormulation = @mLCPdynamics;
        elseif strcmp(formulation,'AP'),           obj.hFormulation = @APdynamics; 
        elseif strcmp(formulation,'NCP'),          obj.hFormulation = @NCPdynamics;   
        elseif strcmp(formulation,'mNCP'),         obj.hFormulation = @mNCPdynamics;  
        elseif strcmp(formulation,'PEG'),          obj.hFormulation = @PEGdynamics;
        elseif strcmp(formulation,'particleLCP'),  obj.hFormulation = @LCPparticleDynamics;
        else   warning('Unrecognized formulation. You can manually add a new formulation with addFormulation(@yourFormulation)');
        end
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addFormulation()
      % Allows user to add a custom formulation handle 
      function addFormulation(obj, formulationHANDLE)
         obj.hFormulation = formulationHANDLE;
         obj.FORMULATION = 'CUSTOM'; 
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% solver()
      % Takes a pre-defined string representing which solver to use
      function obj = solver(obj, solver)
        % Solver handle
        if     strcmp(solver,'PATH'),              obj.hSolver = @wrap_PATH; 
        elseif strcmp(solver,'Lemke'),             obj.hSolver = @wrap_Lemke;       
        elseif strcmp(solver,'fischer_newton'),    obj.hSolver = @wrap_LcpFischerNewton; 
        elseif strcmp(solver,'penalty'),           obj.hSolver = @penalty;
        elseif strcmp(solver,'mlcp_fixed_point'),  obj.hSolver = @wrap_MlcpFixedPoint;
        elseif strcmp(solver,'mncp_fixed_point')  
            obj.hSolver = @wrap_NcpFixedPoint;
            obj.num_fricdirs = 2;
        elseif strcmp(solver,'mncp_fixed_point_pgs')    
            obj.hSolver = @wrap_NcpFixedPoint;
            obj.num_fricdirs = 2;
        elseif strcmp(solver,'mingres'),           obj.hSolver = @mingres;
        else   warning('Unrecognized solver. You can manually add a new solver with addSolver(@yourSolver)');
        end
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% setSolverTolerance()
      % Sets the tolerance for solver convergence.  
      function setSolverTolerance(obj, tol)
         obj.solver.tolerance = tol;  
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addSolver()
      % Allows user to add a custom solver handle 
      function addSolver(obj, solverHANDLE)
         obj.hSolver = solverHANDLE;
         obj.SOLVER = 'CUSTOM'; 
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% addCollisionDetection()
      % Allows user to add a custom collision detection handle 
      function addCollisionDetection(obj, cdHANDLE)
         obj.hCollisionDetection = cdHANDLE;
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% run()
      function obj = run(obj, varargin)
        % Allow user to specify number of iterations on this run
        if ~isempty(varargin)
           if obj.STEP == 0
              obj.MAX_ITERS = varargin{1};
           else
              obj.MAX_ITERS = obj.MAX_ITERS + varargin{1};  
           end
        end
          
        % TODO: Ensure that hFormulation and hSolver are compatible.
        % TODO: Since collision_detection is currently dependent on the
        % order of bodies i.e. cylinders before spheres, consider sorting
        % the list of bodies before running.  Further, static bodies could
        % go at the beginning of the body list, then we can avoid checking
        % static vs. static bounding spheres.
        
        % Collision detection handle
        if strcmp(obj.FORMULATION,'PEG')    % TODO: Only PEG uses alternative CD at present, but I think this will need better handling.  
            obj.hCollisionDetection = @cda_collision_detection;
        else
            obj.hCollisionDetection = @collision_detection;
        end
        
        % Start the simulation
        if obj.GUI.DRAW  
            sim_draw_init( obj );
            sim_run( obj );         % Start GUI. 
        else 
            disp('Running without GUI.  Make sure you have set a reasonable number for Simulation.MAX_ITERS');
            disp('Starting simulation...');
            while obj.STEP < obj.MAX_ITERS   % Relies on user to set a reasonable MAX_ITERS
                obj.step(); 
                disp( ['[' num2str(obj.TIME) ']' 9 'CDtime: ' num2str(obj.timers.collision_detection) ...
                      9 'SOLVEtime: (' num2str(length(obj.Contacts)) ') ' num2str(obj.timers.solver)] );
            end
        end
        
      end % End run()
      
      
      %% Below are some user friendly functions
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% gravityON()
      % Sets an external acceleration of -9.81 in the z direction for each body.
      function obj = gravityON(obj)
          for i=1:length(obj.P)
              if ~obj.P{i}.static
                obj.P{i}.Aext = [0;0;-9.81;0;0;0];   % Will break obj_particle which has Aext = zeros(1,3)
                obj.P{i}.Fext = obj.P{i}.mass * obj.P{i}.Aext;
                % Should probably have a state boolean isGravityOn, and
                % add any existing Fext to the gravitational Fext...
              end
          end
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %% gravityOFF()
      % Removes any external acceleration.
      function obj = gravityOFF(obj)
          for i=1:length(obj.P)
             obj.P{i}.Aext = [0;0;0;0;0;0];
             obj.P{i}.Fext = [0;0;0;0;0;0];
          end
      end
      
      function obj = enableGUI( obj )                       % Enable GUI
         obj.GUI.DRAW = true; 
      end
      function obj = disableGUI( obj )                      % Disable GUI
         obj.GUI.DRAW = false;
      end
      function obj = setRecord( obj, b )                    % Turn recording on/off
         obj.RECORD_DATA = b;         
      end
      function obj = setRecordContacts( obj, b )            % Turn recording of contacts on/off
         obj.RECORD_CONTACTS = b; 
      end
      function obj = setFriction(obj, b)                    % Turn friction on/off
         if ischar(b)
            if strcmpi(b,'on'), obj.FRICTION = true;
            else obj.FRICTION = false; 
            end
         else
            obj.FRICTION = b;
         end 
      end
      
      
      
   end % End methods
   
end % End Simulation


   
   
   
