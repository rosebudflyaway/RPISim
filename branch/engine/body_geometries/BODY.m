%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% BODY.m 
%
% Represents a generic body object.  All other bodies should inherit this. 

classdef BODY < handle
    properties (SetAccess='public')
        
        % Body information 
        name            % Usefule for feedback
        bodyID          % The bodies position in the Simulation.P{} cell array
        body_type       % i.e. mesh, sphere, cylinder, etc.
        geometry        % A body_geometry object containing geom info like verts, radius, etc. 
        
        % Physical properties
        mass            % Mass  (note, changing this will change J)
        J               % Moment of inertia, local frame
        Jworld          % Inertia tensor, in world frame
        Jinv            % Inverse of inertia tensor, in local frame
        JinvWorld       % Inverse of inertia tensor, in world frame
        u               % Position as [x, y, z]  
        quat            % Rotation represented as quaternion
        nu              % Velocity as [Vx, Vy, Vz] & [wx, wy, wz]
        mu              % Friction
        Fext            % External forces, including torques
        Aext            % External acceleration (usually just gravity)
        appliedForces   % Vector of applied forces
        static          % true => Fixed position, false => physical body
        physical        % false => not included in collision detection
        numJoints       % Number of joints

        % Collsion detection 
        bound           % Radius of bounding sphere (spheres aren't ideal)
        BodyIndex       % For indexing in the dynamics formulation 
        ContactCount    % Number of contacts this body has at current iteration
        nonCollisionBods% Vector of bodyIDs that this body does not collide with, e.g. joint bodies. 
        
        % Graphics properties
        graphicsHandle      
        Xdata           % Xdata for drawing
        Ydata           % Ydata for drawing
        Zdata           % Zdata for drawing
        visible
        drawn        	% Whether or not the object has been drawn  
        color           % Color to use when drawing object.
        faceAlpha       % Set alpha (transparency).  Wireframe => 0. 
        edgeAlpha 
        filled        	% Whether to fill object or not

        inUnion         % Boolean, if belongs to union  
        offset          % For use when body is part of a union 
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = BODY()
            
            
            % Initialize physical properties 
            obj.mass = 1;             % Mass  (note, changing this will change J)
            obj.J = eye(3);           % Moment of inertia
            obj.Jinv = eye(3); 
            obj.u = [0; 0; 0];        % Position as [x, y, z]  
            obj.quat = [1 0 0 0]';    % Rotation represented as quaternion
            obj.nu = [0; 0; 0;        % Velocity as [Vx, Vy, Vz] & [wx, wy, wz]
                      0; 0; 0];
            obj.mu = 0.8;             % Friction
            obj.Fext = [0; 0; 0;      % External forces, including torques
                        0; 0; 0];
            obj.Aext = [0; 0; -9.81; % External acceleration (usually just gravity)
                        0; 0; 0];
            obj.appliedForces;        % Vector of applied forces
            obj.static = false;       % true => Fixed position, false => physical body
            obj.physical = true;      % false => not included in collision detection
            obj.numJoints = 0;        % Initially, no joints

            % Collsion detection 
            obj.bound  = 1.4;         % Radius of bounding sphere (spheres aren't ideal)
            obj.BodyIndex = -1;       % For indexing in the dynamics formulation 
            obj.ContactCount = 0;  
            obj.nonCollisionBods = [];

            % Graphics properties    
            obj.visible = true; 
            obj.drawn = false;        % Whether or not the object has been drawn  
            obj.color = 'blue';       % Color to use when drawing object.
            obj.faceAlpha = 0.9;      % Set alpha (transparency).  Wireframe => 0. 
            obj.edgeAlpha = 0.8; 
            obj.filled = 1;           % Whether to fill object or not

            obj.inUnion = 0;          % Boolean, if belongs to union  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Kinematic update
        % Assumes that velocity has already been updated
        % Input: h - The stepsize in seconds (s) 
        function obj = kinematicUpdate( obj, h )
           if obj.static, return; end
           
           % Velocity 
           obj.nu = obj.nu + (obj.Fext/obj.mass)*h;   
           
           % Translation
           obj.u = obj.u + h*obj.nu(1:3);   

           % Rotation 
           w = obj.nu(4:6) * h;            
           wn = norm(w); 
           if wn ~= 0
              obj.quat = qtmultiply( [cos(wn/2); (w/wn)*sin(wn/2)], obj.quat ); % TODO: Is this the correct order?  It works.
           end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Kinematic update with Velocity
        % Assumes that velocity has already been updated
        % Input: h - The stepsize in seconds (s) 
        function obj = kinematicUpdateWithVelocity( obj, newNu, h )
           if obj.static, return; end
            
           % Velocity 
           obj.nu = newNu;   % Assumes Fext was taken into account when determining newNu
           
           % Translation
           obj.u = obj.u + h*newNu(1:3);   

           % Rotation 
           w = newNu(4:6) * h;            
           wn = norm(w); 
           if wn ~= 0
              obj.quat = qtmultiply( [cos(wn/2); (w/wn)*sin(wn/2)], obj.quat ); % TODO: Is this the correct order?  It works.
           end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% massInertiaMatrix()
        function M = massInertiaMatrix( obj )
            if obj.static
               M = zeros(6);
               return;
            end
            
            % Update world frame inertia tensor
            R = qt2rot(obj.quat);
            obj.Jworld = R * obj.J * R';
            M = [ obj.mass*eye(3)   zeros(3)
                     zeros(3)      obj.Jworld ]; 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% applyForce()
        % Apply force F at point p over period dt
        function obj = applyForce( obj, p, F, dt )
            
        end % End applyForce()
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% applyImpulse()
        % Apply impulse P over time dt at point pt (pt is a point in the world frame) 
        function obj = applyImpulse( obj, P, pt, dt )
            
             % Plot for debugging
            hold on; 
            plot3([obj.u(1) pt(1)], [obj.u(2) pt(2)], [obj.u(3) pt(3)], 'g');
            plot3([pt(1) pt(1)+P(1)/3], [pt(2) pt(2)+P(2)/3], [pt(3) pt(3)+P(3)/3], 'r');
            
            % Doesn't affect static bodies
            if obj.static, return; end  
            
            % The linear and rotational components of the impulse
            r = pt - obj.u;  
            I = [     P
                  cross(r,P) ];
            dnu = I*dt*dt/obj.mass;  % TODO: inertia matrix not included? 
            %dnu = massInertiaMatrix \ I * dt*dt; isn't this correct? 
            
            obj.kinematicUpdateWithVelocity(obj.nu+dnu, dt);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% setPosition()
        function obj = setPosition( obj, x,y,z )
            obj.u = [x;y;z];
        end % End setPosition()
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% doNotCollideWith( obj, bodyID )
        function obj = doNotCollideWith(obj, bodyID)
           obj.nonCollisionBods = [obj.nonCollisionBods; bodyID];  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% doesNotCollideWith( obj, bodyID )
        function collides = doesNotCollideWith (obj, bodyID)
           collides = any(obj.nonCollisionBods == bodyID); 
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% draw
%         function obj = draw( obj )
%             % Grab original XYZ data and transform to world frame 
%             X = zeros(size(obj.Xdata));
%             Y = zeros(size(obj.Ydata));
%             Z = zeros(size(obj.Zdata));
%             for i=1:size(X,2)
%                D = qtrotate(obj.quat, [obj.Xdata(:,i) obj.Ydata(:,i) obj.Zdata(:,i)]')';
%                X(:,i) = D(:,1) + obj.u(1);
%                Y(:,i) = D(:,2) + obj.u(2);
%                Z(:,i) = D(:,3) + obj.u(3);
%             end
%             set(obj.graphicsHandle,'XData',X,'YData',Y,'ZData',Z);
%             set(obj.graphicsHandle,'FaceColor',obj.color); 
%         end
        
    end
end










