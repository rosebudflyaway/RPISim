%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_union.m
%
% Generate a union of spheres by filling a big sphere with several small
% ones with overlapping areas
classdef obj_union < handle
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
        
        % Contact variables
        BodyIndex = -1;   % For indexing in the dynamics formulation 
        ContactCount = 0;  
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = obj_union(mass,radius, num, posi)
            obj.u = posi;
            obj.mass = num*mass;
            obj.radius = radius;
            obj.num = num;
            %obj.J = (2/5)*mass*radius^2 * eye(3);
            %obj.bound = 1.1 * radius;
            obj.nSegments = 10;
            r = radius;
            obj.bound = r*3;
            u_sphere = zeros(3, num);
            
            u_sphere(:,1) = obj.u;
            for i = 2:num
                checkingPosition = true;
                while checkingPosition
                    checkingPosition = false;
                    x = (obj.u(1) - 3*r + 1.1*r) + 2*(3*r-1.1*r) * rand(1);
                    y = (obj.u(2) - 3*r + 1.1*r) + 2*(3*r-1.1*r) * rand(1);
                    z = (obj.u(3) - 3*r + 1.1*r) + 2*(3*r-1.1*r) * rand(1);
                    
                    % check the sphere has some overlap with the center one    
                    if(sqrt((obj.u(1)-x)^2 + (obj.u(2)-y)^2 + (obj.u(3)-z)^2) > (1/2)*r && ...
                        sqrt((obj.u(1)-x)^2 + (obj.u(2)-y)^2 + (obj.u(3)-z)^2) < r)
                    
                        for j=2:i-1
                            if sqrt((u_sphere(1, j)-x)^2 + (u_sphere(2,j)-y)^2 + (u_sphere(3, j)-z)^2) > 0.6*r && ...
                                    sqrt((u_sphere(1, j)-x)^2 + (u_sphere(2,j)-y)^2 + (u_sphere(3, j)-z)^2) < 2*r
                                checkingPosition = false;
                                break;
                            else
                                checkingPosition = true;
                            end
                        end
                    else
                        checkingPosition = true;
                    end
                end
                u_sphere(:, i) = [x; y; z];
            end
            % Subtract first column from all the columns.
            % Now each column is the corresponding offset for all the
            % spheres
            offset = bsxfun(@minus, u_sphere, obj.u);
            offset = offset .^2;
            momentum_x = 0;
            momentum_y = 0;
            momentum_z = 0;
            for i=1:obj.num
                    s = obj_sphere(mass, r);
                    s.u = u_sphere(:, i);
                    s.inUnion = 1;
                    s.bound = 1.1*radius;
                    s.offset = s.u - obj.u;
                    s.static = obj.static;
                    s.nu = [0; 0; 0; 0; 0; 0];
                    UN(i) = s;
                    momentum_x = momentum_x + (2/5)*mass*(radius^2) + mass*(offset(2,i)+offset(3,i));
                    momentum_y = momentum_y + (2/5)*mass*(radius^2) + mass*(offset(1,i)+offset(3,i));
                    momentum_z = momentum_z + (2/5)*mass*(radius^2) + mass*(offset(1,i)+offset(2,i));
            end
            obj.J = [momentum_x,      0,          0;
                         0,       momentum_y,     0;
                         0,           0,        momentum_z];
            obj.Jinv = inv(J); 
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
                obj.UNION(i).draw();
            end          
        end
    end
end





