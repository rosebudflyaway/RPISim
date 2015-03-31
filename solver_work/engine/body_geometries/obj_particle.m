%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_particle.m 
%
% A particle object.
% 
classdef obj_particle < handle
    properties (SetAccess='public')
        body_type = 'particle' 
        bodyID
        name            % Useful for feedback
        mass = 1        % Mass  (note, changing this will change J)
        bound = 1000    % Radius of bounding sphere (spheres aren't ideal)
        u = [0; 0; 0]   % Position of center of mass as [x, y, z]  
        nu = [0; 0; 0]  % Velocity as [Vx, Vy, Vz] 
        mu = 0.5        % Friction
        static = 0      % 1=>Fixed position, 0=> physical body
        Fext = [0; 0; 0]% External forces
        Aext = [0; 0; -9.81]% External acceleration (usually just gravity). 
            
        visible = true;  
        color = 'red'   % Color to use when drawing object.
        drawn = 0       % Whether or not the object has been drawn
        graphicsHandle  % graphics handle
        
        % Contact variables
        BodyIndex = -1;   % For indexing in the dynamics formulation 
        ContactCount = 0;  
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = obj_particle( pos ) 
            obj.u = pos; 
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% setStatic()
        function obj = setStatic(obj, s)
            obj.static = s;
            if obj.static
                obj.color = [.66 .71 .69];  % I just like this color for static objs (jw)
            else
                obj.color = rand(1,3);  % Random color for dynamic objects
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw mesh
        function draw( obj )
            if ~obj.visible, return; end
            
            if obj.drawn == 0
               
                obj.graphicsHandle = plot3(obj.u(1),obj.u(2),obj.u(3),'o','MarkerFaceColor',obj.color); 
                obj.drawn = 1;
                
            else
                
                if obj.static == 0  % Don't redraw static objects.
                   set( obj.graphicsHandle,'XData',obj.u(1), ...
                                           'YData',obj.u(2), ...
                                           'ZData',obj.u(3) );
                end
            end
        end
        
    end
end

