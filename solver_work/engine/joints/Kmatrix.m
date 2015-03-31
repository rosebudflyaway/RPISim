  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Kmatrix.m
%
% Calculates K matrix from Bender's method for two bodies joined by 'joint'
% Method found in Jan's post at 
% http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=4&t=878
% 
% The following calculation of K includes an optimization of the original
% formula:    K = (1/body.mass)*eye(3) - hat(r1)*inv(body.J)*hat(r1)
%
function [K1 K2] = Kmatrix(joint) 
    body1 = joint.body1;
    body2 = joint.body2; 

    % K1, first body
    if body1.static
        K1 = zeros(3);
    else
        K1 = zeros(3); 
        r1 = joint.p1 - body1.u;
        a = r1(1);
        b = r1(2);
        c = r1(3);
        J = body1.Jinv;  % TODO: J is in BODY FRAME
        j1 = J(1,1);
        j2 = J(1,2);
        j3 = J(1,3);
        j4 = J(2,1);
        j5 = J(2,2);
        j6 = J(2,3);
        j7 = J(3,1);
        j8 = J(3,2);
        j9 = J(3,3);
        m = 1/body1.mass; 

        K1(1,1) = c*c*j5 - b*c*(j6+j8) + b*b*j9+m;
        K1(1,2) = -(c*c*j4) + a*c*j6 + b*c*j7 - a*b*j9;
        K1(1,3) = b*c*j4 - a*c*j5 - b*b*j7 + a*b*j8;
        K1(2,1) = -(c*c*j2) + b*c*j3 + a*c*j8 - a*b*j9;
        K1(2,2) = c*c*j1 - a*c*(j3 + j7) + a*a*j9 + m;
        K1(2,3) = -(b*c*j1) + a*c*j2 + a*b*j7 - a*a*j8;
        K1(3,1) = b*c*j2 - b*b*j3 - a*c*j5 + a*b*j6;
        K1(3,2) = -(b*c*j1) + a*b*j3 + a*c*j4 - a*a*j6;
        K1(3,3) = b*b*j1 - a*b*(j2+j4) + a*a*j5 + m; 
    end
    % K2, second body
    if body2.static
        K2 = zeros(3);
    else
        K2 = zeros(3); 
        r2 = joint.p2 - body2.u;
        a = r2(1);
        b = r2(2);
        c = r2(3);
        J = body2.Jinv; 
        j1 = J(1,1);
        j2 = J(1,2);
        j3 = J(1,3);
        j4 = J(2,1);
        j5 = J(2,2);
        j6 = J(2,3);
        j7 = J(3,1);
        j8 = J(3,2);
        j9 = J(3,3);
        m = 1/body2.mass; 
        K2(1,1) = c*c*j5 - b*c*(j6+j8) + b*b*j9+m;
        K2(1,2) = -(c*c*j4) + a*c*j6 + b*c*j7 - a*b*j9;
        K2(1,3) = b*c*j4 - a*c*j5 - b*b*j7 + a*b*j8;
        K2(2,1) = -(c*c*j2) + b*c*j3 + a*c*j8 - a*b*j9;
        K2(2,2) = c*c*j1 - a*c*(j3 + j7) + a*a*j9 + m;
        K2(2,3) = -(b*c*j1) + a*c*j2 + a*b*j7 - a*a*j8;
        K2(3,1) = b*c*j2 - b*b*j3 - a*c*j5 + a*b*j6;
        K2(3,2) = -(b*c*j1) + a*b*j3 + a*c*j4 - a*a*j6;
        K2(3,3) = b*b*j1 - a*b*(j2+j4) + a*a*j5 + m; 
    end

end
    
    
    