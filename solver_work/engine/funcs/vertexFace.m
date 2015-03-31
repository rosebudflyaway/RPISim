function [ cexists loc n psi ] = vertexFace( v, p1,p2,p3,p4,dir, dist )
%POINTPLANE Location and distance of contact between a vertex and a face
%   v is the vertex as a 3x1 vector
%   p1 - p4 are the four corners of the face in counter clockwise order
%   dist is the distance used to determine if a contact should be
%   created.

    cexists = 0;
    loc = [0;0;0];
    psi = 0;
    
    n = cross(p2-p1,p3-p2);
    n = n/norm(n);
    
    % If this face is in the direction of the other pin
    if dot(dir,n) > 0
        % Vector from point on plane to the vertex
        w = v - p1;

        % Distance from plane to point
        psi = dot(n,w);

        % Vertex is breaking half plane, so check if in bounds
        if psi < dist
          % Find the midpoints on all four sides
          m1 = (p1+p2)/2;
          m2 = (p2+p3)/2;
          m3 = (p3+p4)/2;
          m4 = (p4+p1)/2;

          % Four 'normal' vectors to the lines. They are perpendicular to the
          % edges of the quad pointing in towards the plane formed by the four
          % points
          n1 = cross(n,p2-p1);% n1 = n1/norm(n1);  % Don't need to normalize
          n2 = cross(n,p3-p2);% n2 = n2/norm(n2);  % since we are only looking
          n3 = cross(n,p4-p3);% n3 = n3/norm(n3);  % for the sign
          n4 = cross(n,p1-p4);% n4 = n4/norm(n4);

          a = dot(n1,v-m1);
          b = dot(n2,v-m2);
          c = dot(n3,v-m3);
          d = dot(n4,v-m4);

          % If all dot products are positive, then the vertex is colliding
          % with the plane
          if  a > 0 && b > 0 && c > 0 && d > 0
            % Contact, now must determine location on plane
            loc = v - psi*n;
            cexists = 1;
          end

        end
    end
end

