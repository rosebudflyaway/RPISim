%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_union_foot.m
%
% six spheres on a layer to represent the foot
classdef obj_union_foot < handle
    properties (SetAccess='public')
        body_type = 'union'
        name            % Usefule for feedback
        UNION
        %obj = obj_sphere
        %num = 3
        mass = 1        % Mass  (note, changing this will change J)
        num = 6        % the number of spheres to form the union
        J = eye(3)      % Moment of inertia
        radius = 0.025      % Radius of sphere
        bound = 0.35     % Radius of bounding sphere (spheres aren't ideal)
        u = [0; 0; 0]   % Position as [x, y, z]
        quat = [1 0 0 0] % Rotation represented as quaternion
        nu = [0; 0; 0;  % Velocity as [Vx, Vy, Vz] & [wx, wy, wz]
              0; 0; 0]
        
        mu = 0.5        % Friction
        %UNION = obj_sphere(m, r);
        %mu = 0
        %mu = 0.08
        static = 0      % 1=>Fixed position, 0=> physical body
        Fext_constant = [0;0;0;0;0;0]; % External force 
        Fext = [0; 0; 0;% External forces, including torques
                0; 0; 0]
        color = 'green' % Color to use when drawing object.
        filled = 1      % Whether to fill object or not
        drawn = 0       % Whether or not the object has been drawn
        visible = 1;
        nSegments       % Number of segments to draw the sphere with
        
        appliedForces
        reset_vel
        tune_torque = [0; 0; 0]
         
        ref_point
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = obj_union_foot(mass,radius, num, posi)
            obj.u = posi;
            obj.mass = num*mass;
            obj.radius = radius;
            obj.num = num;
            %obj.J = (2/5)*mass*radius^2 * eye(3);
            %obj.bound = 1.1 * radius;
            obj.nSegments = 10;
            r = radius;
            obj.bound = radius*3;
             
%             for i = 1:num
%                 u_spherex(i) = obj.u(1)-(num-1)*r + (i-1)*2*r;
%             end
            u_spherex = [obj.u(1)-1.8*r; obj.u(1); obj.u(1)+1.8*r; obj.u(1)-1.8*r; obj.u(1); obj.u(1)+1.8*r];
            u_spherey = [obj.u(2)+0.9*r; obj.u(2)+0.9*r; obj.u(2)+0.9*r; obj.u(2)-0.9*r; obj.u(2)-0.9*r; obj.u(2)-0.9*r;];
            u_spherez = obj.u(3) * diag(eye(num));
            xoffset = zeros(num, 1);
            yoffset = zeros(num, 1);
            zoffset = zeros(num, 1);
            
            for i = 1:num
                xoffset(i) = norm((u_spherex - obj.u(1)))^2;
                yoffset(i) = norm((u_spherey - obj.u(2)))^2;
                zoffset(i) = norm((u_spherez - obj.u(3)))^2;
            end
            
            %UN = [];
            momentum_x = 0;
            momentum_y = 0;
            momentum_z = 0;
            
            for i=1:obj.num
                    s = obj_sphere(mass, r);
                    s.u = [u_spherex(i); u_spherey(i); obj.u(3)];
                    s.inUnion = 1;
                    s.bound = 1.1*radius;
                    s.offset = s.u - obj.u;
                    s.static = obj.static;
                    s.nu = [0; 0; 0; 0; 0; 0];
                    UN(i) = s;
                    momentum_x = momentum_x + (2/5)*mass*(radius^2) + mass*(yoffset(i) + zoffset(i));
                    momentum_y = momentum_y + (2/5)*mass*(radius^2) + mass*(zoffset(i) + xoffset(i));
                    momentum_z = momentum_z + (2/5)*mass*(radius^2) + mass*(xoffset(i) + yoffset(i));
            end
            obj.J = [momentum_x,      0,          0;
                         0,       momentum_y,     0;
                         0,           0,        momentum_z];
            obj.Jinv = inv(obj.J); 
            obj.UNION = UN;
            obj.appliedForces = [];  % Not technically necessary
            obj.tune_torque = [0; 0; 0];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% applyForce()  % Should be updated to include torques, or there should be applyTorque()
        % p: point in LOCAL frame of reference
        % F: 3x1 vecotr of WORLD frame force(xyz) and torque(xyz)
        % t0: start time to apply force
        % tf: end time to apply force
        function obj = applyForce( obj, p, F, t0, tf )
            obj.appliedForces(length(obj.appliedForces) + 1).p = p;
            obj.appliedForces(length(obj.appliedForces)).F = F;
            obj.appliedForces(length(obj.appliedForces)).t0 = t0;
            obj.appliedForces(length(obj.appliedForces)).tf = tf;
        end % End applyForce()
        
        %% Reset the velocity during the simulation, say when the foot touch the terrain, set the velocity equal to zero
        function obj = Reset_velocity( obj, v, t0, tf)
             obj.reset_vel(length(obj.reset_vel) + 1).vel = v;
             obj.reset_vel(length(obj.reset_vel)).t0 = t0;
             obj.reset_vel(length(obj.reset_vel)).tf = tf;
        end
        
        %% applyTorque() % apply the torque when the shoe is not balanced
        function obj = TuneTorque(obj, torque)
            obj.tune_torque = obj.tune_torque + torque;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw sphere
        function obj = draw( obj )
            if obj.visible == 0, return; end
            
            for i = 1:obj.num
                obj.UNION(i).color = 'red';
                obj.UNION(i).faceAlpha = 1;
                obj.UNION(i).draw();
            end          
        end
    end
end






