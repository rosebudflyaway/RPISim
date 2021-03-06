%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_sphere.m 
%
% Represents a sphere object
classdef bodySphere < BODY
    properties (SetAccess='public')  
        radius = 1      % Radius of sphere
        nSegments       % Number of segments to draw the sphere with
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        function obj = bodySphere(mass,radius) 
          obj.body_type = 'sphere'; 
          obj.mass = mass;
          obj.radius = radius;
          obj.bound = radius;
          obj.J = (2/5)*mass*radius^2 * eye(3);
          obj.Jinv = inv(obj.J); 
          obj.bound = 1.1 * radius;
          obj.nSegments = 10;
          obj.Fext = obj.mass * obj.Aext; 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw sphere
        function obj = draw( obj )
            
            if ~obj.drawn
                obj.drawn = true;
                [obj.Xdata,obj.Ydata,obj.Zdata] = sphere(obj.nSegments-1);
                
                obj.Xdata=obj.Xdata*obj.radius;
                obj.Ydata=obj.Ydata*obj.radius;
                obj.Zdata=obj.Zdata*obj.radius;
            
                obj.graphicsHandle = surf( obj.Xdata, ...  
                                           obj.Ydata, ...
                                           obj.Zdata );
               set(obj.graphicsHandle,'FaceColor',obj.color); 
               set(obj.graphicsHandle,'FaceAlpha',obj.faceAlpha); 
            end

            [x y z] = obj.getRotatedMesh();
            set(obj.graphicsHandle,'Xdata',x,'Ydata',y,'Zdata',z);
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% GetRotatedMesh
        % Retuns the the mesh in the world frame
        function [x y z] = getRotatedMesh(obj)
            x = zeros(obj.nSegments,size(obj.Xdata,2));
            y = zeros(obj.nSegments,size(obj.Ydata,2));
            z = zeros(obj.nSegments,size(obj.Zdata,2));
            % Rotate the mesh for drawing
            for i = 1:obj.nSegments
                ptNew = [obj.Xdata(i,:); obj.Ydata(i,:); obj.Zdata(i,:)]';
                ptRot = quatrotate(obj.quat,ptNew); ptRot = ptRot';
                x(i,:) = ptRot(1,:) + obj.u(1);
                y(i,:) = ptRot(2,:) + obj.u(2);
                z(i,:) = ptRot(3,:) + obj.u(3);
            end
        end
    end
end
