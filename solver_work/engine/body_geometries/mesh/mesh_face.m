%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_face.m 
%
% Represents the face of a mesh object
classdef mesh_face < handle
    properties (SetAccess='public')
        verts           % Vector of indices referencing verts in face
                          % IMPORTANT!: It is assumed that this list of verts
                          % is counter-clockwise (viewed from outside face)
        num_verts       % Number of vertices in the face
        normal          % Column vector representing normal direction
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        % Takes a Nx3 array of vertices N x (x,y,z)
        function obj = mesh_face(verts)
          obj.verts = verts; 
          obj.num_verts = max(size(verts)); 
        end
     
    end
end

