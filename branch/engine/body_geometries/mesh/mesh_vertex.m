%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_vertex.m 
%
% Represents the vertex of a mesh object
% Every vertex has [x;y;z] coordinates, as well as a list of
% indexes to each face it belongs to.  
classdef mesh_vertex < handle
    properties (SetAccess='public') 
        local_coords    % Coordinates as [x;y;z] 
        world_coords    % local_coordinates transformed into world frame
        faces           % Vector of face indices that the vertex belongs to
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        function obj = mesh_vertex(coords,faces) 
            % Assert that coords is a column vector
            if size(coords,1) == 3
                obj.local_coords = coords; 
            else
                obj.local_coords = coords';
            end
            obj.world_coords = obj.local_coords;  % Initial, world_coords = local_coords
            obj.faces = faces;
        end
        
        % Overload ==
        function out = eq(a, b)
            % compare the X,Y values
        
            out = all((a.local_coords == b.local_coords) & (a.world_coords == b.world_coords));
        
        end
        
        % Overload disp
        function disp(a)
        
            disp(sprintf('[[[ VERTEX ]]]\nLocal coordinates: [%5.3f; %5.3f; %5.3f]\nWorld coordinates: [%5.3f; %5.3f; %5.3f]\n', a.local_coords(1), a.local_coords(2), a.local_coords(3), a.world_coords(1), a.world_coords(2), a.world_coords(3)));
        
        end
        
        
        % Shortcut methods for accesing coordinates
        function X = x(obj)
           X = obj.local_coords(1); 
        end
        function Y = y(obj)
           Y = obj.local_coords(2); 
        end
        function Z = z(obj)
           Z = obj.local_coords(3); 
        end
        
    end
end

