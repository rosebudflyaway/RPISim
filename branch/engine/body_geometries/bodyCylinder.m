%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_cylinder.m 
%
% Represents a cylinder object
classdef bodyCylinder < BODY
    properties (SetAccess='public')
        radius = 1              % Radius of cylinder
        height = 1              % Height of cylinder (total height) 
        graphicsHandleTop       
        graphicsHandleBottom
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        function obj = bodyCylinder(mass, radius, height)
           obj.body_type = 'cylinder'; 
           obj.mass = mass;
           obj.radius = radius;
           obj.height = height; 
           obj.J = [(1/12)*mass*(3*radius^2 + height^2)     0       0
                    0       (1/12)*mass*(3*radius^2 + height^2)     0
                    0       0                       (mass*radius^2)/2];
           obj.Jinv = inv(obj.J); 
           obj.bound = 1.1 * max(obj.height,obj.radius);   % Not the best bound
           obj.color = rand(1,3);  
           
           obj.Fext = obj.mass * obj.Aext;  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw cylinder
        function obj = draw( obj )
            if obj.visible == 0, return; end
            
            if obj.drawn == 0
               obj.drawn = 1; 
               [obj.Xdata,obj.Ydata,obj.Zdata] = cylinder(obj.radius);
               obj.Zdata = obj.Zdata*obj.height - obj.height/2;

               % Cylinder
               obj.graphicsHandle = surf( obj.Xdata, ...
                                          obj.Ydata, ...
                                          obj.Zdata );
                                      
               % Caps, top and bottom
               obj.graphicsHandleTop = patch(obj.Xdata(2,:),obj.Ydata(2,:),obj.Zdata(2,:),obj.color);
               obj.graphicsHandleBottom = patch(obj.Xdata(1,:),obj.Ydata(1,:),obj.Zdata(1,:),obj.color);
               
               % Graphics properties
               set(obj.graphicsHandle,'FaceColor',obj.color,'EdgeAlpha',obj.faceAlpha,'FaceAlpha',0.5);  % TODO: separate face & edge alphas
               set(obj.graphicsHandleTop,'FaceColor',obj.color,'EdgeAlpha',obj.faceAlpha,'FaceAlpha',0.5);
               set(obj.graphicsHandleBottom,'FaceColor',obj.color,'EdgeAlpha',obj.faceAlpha,'FaceAlpha',0.5);
            end
            
            x1=obj.Xdata(1,:);  x2=obj.Xdata(2,:);
            y1=obj.Ydata(1,:);  y2=obj.Ydata(2,:);
            z1=obj.Zdata(1,:);  z2=obj.Zdata(2,:);

            n1 = qtrotate(obj.quat, [x1;y1;z1]);   %n1=n1';
            n2 = qtrotate(obj.quat, [x2;y2;z2]);   %n2=n2';

            x = [n1(1,:);n2(1,:)] + obj.u(1);
            y = [n1(2,:);n2(2,:)] + obj.u(2);
            z = [n1(3,:);n2(3,:)] + obj.u(3);

            set(obj.graphicsHandle,'Xdata',x,'Ydata',y,'Zdata',z);
            set(obj.graphicsHandleTop,'Xdata',x(2,:),'Ydata',y(2,:),'Zdata',z(2,:));
            set(obj.graphicsHandleBottom,'Xdata',x(1,:),'Ydata',y(1,:),'Zdata',z(1,:));
        end
    end
end

