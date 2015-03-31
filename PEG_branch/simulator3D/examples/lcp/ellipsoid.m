function [xx,yy,zz] = ellipsoid(n_faces, origin, radius, euler_p)
%function [xx,yy,zz] = ellipsoid(n,d,r,e)
%ELLIPSIOD:  Generate ellipsoid.
%   [X,Y,Z] = ELLIPSOID(n_faces,origin,radius) with origin=[0 0 0] and
%   radius=[1 1 1] generates three (n_faces+1)-by-(n_faces+1) matrices so
%   that SURF(X,Y,Z) produces a sphere of radius 1 centered at the origin.
%
%   [X,Y,Z] = ELLIPSOID uses n_faces = 20, origin=[0 0 0] and radius=[1 1 1].
%
%   ELLIPSOID(n_faces) and just ELLIPSOID graph the ellipsoid as a SURFACE
%   and do not return anything.
%
%   See also CYLINDER and MY_SPHERE.

%   Clay M. Thompson 4-24-91, CBM 8-21-92.
%   Copyright (c) 1984-97 by The MathWorks, Inc.
%   $Revision: 5.2 $  $Date: 1997/04/08 06:48:02 $
%
%   Revised by Jeff Trinkle to move the center and to strecth and
%   squash it to generate an ellipsoid.  Also, added ability to 
%   reorient the ellipsoid. 

if nargin == 0
   n_faces = 20; origin = [0 0 0]; radius = [1 1 1]; euler_p = [1 0 0 0];
elseif nargin == 1
   origin = [0 0 0]; radius = [1 1 1]; euler_p = [1 0 0 0];
elseif nargin == 2
   radius = [1 1 1]; euler_p = [1 0 0 0];
elseif nargin == 3
   euler_p = [1 0 0 0];
elseif nargin == 4
   ;
else
   display('Too many arguments passed to ellipsoid.m')
end

% If euler_p is a rotation matrix, then it is R (used below).
se = size(euler_p);
if se(1) == 3 & se(2) == 3
    I = euler_p' * euler_p;
    residual = I - eye(3,3);
    norm_res = norm(residual);
    if (norm_res > 1e-6)
        fprintf('\nOrientation matrix is not orthogonal.  Continuing anyway!!!!');
        R = euler_p;
    else
        R = euler_p;
    end
elseif (se(1) == 4 & se(2) == 1) | (se(1) == 1 & se(2) == 4)
    R = ep2rot(euler_p);
else
    fprintf('\nOrientation must be specified as a 3x3 rotation matrix');
    fprintf('\n  or a 4x1 or 1x4 vector of Euler parameters!!');
    fprintf('\n  Continuing anyway, but with identity orientation.');
    R = eye(3);
end
        
% Create sample directions in the unit sphere
theta = (-n_faces:2:n_faces)/n_faces*pi;           % Array from -pi to pi
phi = (-n_faces:2:n_faces)'/n_faces*pi/2;          % Array from -pi/2 to pi/2
cosphi = cos(phi); cosphi(1) = 0; cosphi(n_faces+1) = 0;
sintheta = sin(theta); sintheta(1) = 0; sintheta(n_faces+1) = 0;

% Create points on the ellipsoid (body-fixed frame)
x = radius(1) * cosphi*cos(theta);
y = radius(2) * cosphi*sintheta;
z = radius(3) * sin(phi)*ones(1,n_faces+1);

if nargin == 4
   np1sqrd = (n_faces+1) * (n_faces+1);
   pts = [reshape(x,1,np1sqrd); reshape(y,1,np1sqrd); reshape(z,1,np1sqrd)];
   new_pts = R * pts;
   x_rotd = reshape(new_pts(1,:), n_faces+1, n_faces+1);
   y_rotd = reshape(new_pts(2,:), n_faces+1, n_faces+1);
   z_rotd = reshape(new_pts(3,:), n_faces+1, n_faces+1);
   
   % Shift the points by the displacement, origin.
   x_tformd = x_rotd + origin(1);
   y_tformd = y_rotd + origin(2);
   z_tformd = z_rotd + origin(3);
else
   % Shift the points by the displacement, origin.
   x_tformd = x + origin(1);
   y_tformd = y + origin(2);
   z_tformd = z + origin(3);
end

if nargout == 0
   figure;
   axis equal;
   surf(x_tformd,y_tformd,z_tformd);
else
   xx = x_tformd; yy = y_tformd; zz = z_tformd;
end
