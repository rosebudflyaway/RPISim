 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% contact.m 
%
% Represents a contact between two objects
classdef contact < handle
    properties (SetAccess='public')
        % NOTE: Several of these attributes may become vectors or matrices when 
        %       colliding polyhedra since CDA allows multiple facets per
        %       contact.  E.g. normal, tangent, & psi_n will containt
        %       multiple normals, tangents, and distances.  
        id          % Contact ID
        body_1      % Index of first body in contact
        body_2      % Index of second body in contact
        normal      % Normal direction (from Body_1 to Body_2)
        tangent     % Tangent direction
        
        %% Objects including sphere and cylinder use the following fields
        p1          %[x;y;z] Vector from body_1 COM to contact point1 in WORLD frame     
        p2          %[x;y;z] Vector from body_2 COM to contact point2 in WORLD frame
        psi_n       % Penetration depth 
        
        %% The following fields are included for use with the CDA Method
        %  and intended to be used with poly_m'PATH_LICENSE_STRING'esh type objects.  
        collision_type  % E.g. 'vert-face', 'edge-edge' (maybe not necessary)
        %body_1_point   % Point of contact on body1 (could still just use p1 from above)
        body_2_facets   % A vector of indices representing the facets (probably 
                        % faces) on body 2.  For example, [1 3 4]' would
                        % represent the first, third, and fourth faces.  
        
    end
    
    methods
        % Constructor
        function obj = contact(id, b1, b2, n, t, p1, p2, psi_n)
           obj.id = id; 
           obj.body_1 = b1;
           obj.body_2 = b2;
           obj.normal = n;
           obj.tangent = t;
           obj.p1 = p1;
           obj.p2 = p2;
           obj.psi_n = psi_n; 
           obj.collision_type = '';
        end

    end
    
end
 