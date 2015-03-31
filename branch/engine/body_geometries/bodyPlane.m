%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% bodyPlane.m 
%
% Represents a plane body
classdef bodyPlane < BODY
    properties (SetAccess='public')  
        p               % 1x3, A point on the plane
        n               % 1x3, A unit normal vector 
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = bodyPlane(p,n) 
          obj.body_type = 'plane'; 
          obj.static = true;            % Planes must ALL be static
          obj.u = p;
          obj.n = n/norm(n);
          obj.faceAlpha = 0.4; 
          obj.bound = inf;              % Planes have infinite bounding spheres... for now
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw plane
        function obj = draw( obj )
            if obj.drawn == 0
               obj.drawn = 1;
               scl = 5;
                
               t1 = arbitraryTangent(obj.n);  % An arbitrary tangent
               t2 = rot(obj.n,pi/2)*t1;
               t3 = rot(obj.n,pi/2)*t2;
               t4 = rot(obj.n,pi/2)*t3; 
               
               X = obj.u(1)+scl*[t1(1) t2(1) t3(1) t4(1)];
               Y = obj.u(2)+scl*[t1(2) t2(2) t3(2) t4(2)];
               Z = obj.u(3)+scl*[t1(3) t2(3) t3(3) t4(3)];
               obj.graphicsHandle = patch('XData',X,'YData',Y,'ZData',Z); 
                
               set(obj.graphicsHandle,'FaceColor',obj.color); 
               set(obj.graphicsHandle,'FaceAlpha',obj.faceAlpha); 
            end

            %set(obj.graphicsHandle,'Xdata',x,'Ydata',y,'Zdata',z);
        end
        
        
    end
end
