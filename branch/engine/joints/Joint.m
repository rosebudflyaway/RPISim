%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Joint.m 
%
% Represents a joint between two bodies.  
% The possible joint types are:
%       'fixed'
%       'revolute'
%       'prismatic'
%       'cylindrical'
%       'spherical'
%
% An important convention is that Z is always the "joint axis."  That is,
% revolute joints rotate about Z, prismatic joints translate along Z, etc.

classdef Joint < handle
   properties (SetAccess='public') 
       jointID      % A Unique ID
       type         % Type of joint
       jntCode      % Integer code representing type
       mask         % A column mask for selecting the joint type
       body1        % Pointer to body1
       body2        % Pointer to body2
       constraintIndex   % Column position of this joint in the bilateral constraint matrix
       numConstraints     % The number of constraints this joint has
       numPreviousConstraints % The total joint constraints for all previous joints 
       
       pos          % Initial world position of joint when initialized
       Xdir         % Initial x-direction of joint
       Ydir         % Initial y-direction 
       Zdir         % Initial z-direction (always considered the AXIS of the joint)
       theta        % The current value of the joint "angle" 
       theta_prev
       
       linkedJoints % List of jointIDs of joints that are in a chain with this joint
       
       mu           % Joint friction
       
       quat1        % Quaternion orientation of joint frame from body1 to joint
       jointPos1    % Position of joint frame from body1 COM to joint
       T1           % Transformation matrix from body1 COM to joint on body1
       T1world
       
       quat2        % Quaternion orientation of joint frame from body1 to joint
       jointPos2    % Position of joint frame from body1 COM to joint
       T2           % Transformation matrix from body2 COM to joint on body2
       T2world
       
       P1           % Position of joint as body1 sees it, in WORLD coordinates
       X1           % Position of X=1 on body1's joint frame, in WORLD coordinates
       Y1           % Position of Y=1
       Z1           % Position of Z=1
       V1
       
       P2           % Position of joint as body2 sees it, in WORLD coordinates
       X2           % Position of X=1 on body2's joint frame, in WORLD coordinates
       Y2           % Position of Y=1
       Z2           % Position of Z=1
       V2
              
       % Graphics data
       drawn
       P1Handle
       P1_X_AxisHandle
       P1_Y_AxisHandle
       P1_Z_AxisHandle
       P1_Z_pointHandle
       P2Handle
       P2_X_AxisHandle
       P2_Y_AxisHandle
       P2_Z_AxisHandle
       P2_Z_pointHandle
       labelHandle
       
       % For joint corrections
       bender       % A struct for storing bender data
   end
   
   methods
       %% Constructor
       function obj = Joint( type, body1, pos1, quat1, body2, pos2, quat2, npc )
           obj.type = type;         
           
           obj.body1 = body1;
           obj.jointPos1 = pos1;
           obj.quat1 = quat1;
           obj.body2 = body2;
           obj.jointPos2 = pos2;
           obj.quat2 = quat2; 
           obj.initTransformations(); 
           
           obj.update(); 
           obj.pos = obj.P1;  % Since bodies have not yet moved, p1 = p2
           obj.Xdir = obj.X1 - obj.P1;
           obj.Ydir = obj.Y1 - obj.P1;
           obj.Zdir = obj.Z1 - obj.P1; 
           
           obj.linkedJoints = []; 
           obj.mu = body1.mu * body2.mu; 
           obj.drawn = false; 
           
           % Mask columns to obtain specific joint types
           if strcmp(obj.type, 'fixed')
               obj.mask = [1 2 3 4 5 6];    obj.jntCode = 1;
           elseif strcmp(obj.type, 'revolute')
               obj.mask = [1 2 3 4 5  ];    obj.jntCode = 2;
           elseif strcmp(obj.type, 'prismatic')
               obj.mask = [1 2   4 5 6];    obj.jntCode = 3;
           elseif strcmp(obj.type, 'cylindrical')
               obj.mask = [1 2   4 5  ];    obj.jntCode = 4;
           elseif strcmp(obj.type, 'spherical')
               obj.mask = [1 2 3      ];    obj.jntCode = 5;
           end
           obj.numPreviousConstraints = npc; 
           obj.numConstraints = length(obj.mask); 
           
           obj.theta = 0;
           obj.theta_prev = 0;
       end
       
       %% initTransformations()
       function obj = initTransformations( obj )
           obj.T1 = zeros(4,4);
           obj.T1(4,4) = 1;
           obj.T1(1:3,1:3) = qt2rot(obj.quat1);
           obj.T1(1:3,4) = obj.jointPos1; 
           
           obj.T2 = zeros(4,4); 
           obj.T2(4,4) = 1;
           obj.T2(1:3,1:3) = qt2rot(obj.quat2);
           obj.T2(1:3,4) = obj.jointPos2;
       end
       
       %% update() 
       % Updates values affected by motion of bodies 1 & 2
       function obj = update( obj )
           % Update body joint frames 
           T = zeros(4,4); 
           T(4,4) = 1;
           T(1:3,1:3) = qt2rot(obj.body1.quat);
           T(1:3,4) = obj.body1.u; 
           T = T*obj.T1;
           obj.P1 = T(1:3,4);
           obj.X1 = T*[1;0;0;1]; obj.X1 = obj.X1(1:3);
           obj.Y1 = T*[0;1;0;1]; obj.Y1 = obj.Y1(1:3);
           obj.Z1 = T*[0;0;1;1]; obj.Z1 = obj.Z1(1:3);
           obj.T1world = T;     % Stores T from world to joint for later use
           
           T = zeros(4,4); 
           T(4,4) = 1;
           T(1:3,1:3) = qt2rot(obj.body2.quat);
           T(1:3,4) = obj.body2.u; 
           T = T*obj.T2;
           obj.P2 = T(1:3,4);
           obj.X2 = T*[1;0;0;1]; obj.X2 = obj.X2(1:3);
           obj.Y2 = T*[0;1;0;1]; obj.Y2 = obj.Y2(1:3);
           obj.Z2 = T*[0;0;1;1]; obj.Z2 = obj.Z2(1:3);
           obj.T2world = T; 
           
           % Determine the current value of theta
           %obj.theta_prev = obj.theta;
           if obj.jntCode == 2  % Revolute
               z1 = obj.X1 - obj.P1;
               z2 = obj.X2 - obj.P2; 
               obj.theta = real( acos( dot(z1,z2) ) );
               if dot( obj.Y1-obj.P1 , obj.X2-obj.X1 ) < 0
                  obj.theta = -obj.theta;  
               end
           elseif obj.jntCode == 3 % Prismatic
               obj.theta = norm(obj.P2 - obj.P1); 
           else
               %warning('Only revolute joints are currently functional.');
           end
       end
       
       
       %% The joint constraint Jacobians (Gc, or Gn in the dynamics)
       % Returns the constraint jacobian, broken into body1 and body2 
       function [G1c G2c] = Jacobians( obj )
           
           p1 = obj.P1; 
           x1 = obj.X1 - p1;
           y1 = obj.Y1 - p1;
           z1 = obj.Z1 - p1;
           r1 = p1 - obj.body1.u;
           
           p2 = obj.P2;
           x2 = obj.X2 - p2;
           y2 = obj.Y2 - p2;
           z2 = obj.Z2 - p2;
           r2 = p2 - obj.body2.u; 

           zrs = zeros(3,1); 
           
           % Not compuationally efficient, but let's write the whole thing
           if obj.body1.static
               G1c = zeros(6);
           else
               G1c = [      x1               y1             z1        zrs   zrs   zrs 
                      cross3(r1,x1)   cross3(r1,y1)   cross3(r1,z1)   x1    y1    z1  ];
           end
           if obj.body2.static
               G2c = zeros(6);
           else
               G2c = [     -x2              -y2              -z2          zrs    zrs    zrs 
                      cross3(r2,-x2)   cross3(r2,-y2)   cross3(r2,-z2)   -x2    -y2    -z2  ];
           end
                  
           % Apply mask
           G1c = G1c(:,obj.mask);
           G2c = G2c(:,obj.mask); 
              
       end % End Jacobian()
       
       %% The joint constraint error  
       % Input:  h - the time step
       % Output: C - the joint's constraint position error
       %         Cdot - the constraint velocity error
       % TODO: it is more efficient to combine this along with Jacobians() above.
       function [C Cdot] = constraintError( obj )
 
           % Position error
           C = [obj.P1-obj.P2; cross3(obj.Z1-obj.P1,obj.Z2-obj.P2)];  % TODO: I don't think the second half is correct
           C = C(obj.mask); 
           
           % Velocity error
           [G1c G2c] = Jacobians( obj );
           if obj.body1.static
              Cdot = G2c' * obj.body2.nu;
           elseif obj.body2.static
              Cdot = G1c' * obj.body1.nu;
           else
              Cdot = [G1c;G2c]' * [obj.body1.nu; obj.body2.nu];
           end 
           
       end % End constraintError()
       
       
       %% draw()
       % Draws an axis, representing the joint
       function obj = draw( obj )
           
           p1 = obj.P1; 
           x1 = obj.X1;
           y1 = obj.Y1;
           z1 = obj.Z1; 
           p2 = obj.P2;
           x2 = obj.X2;
           y2 = obj.Y2;
           z2 = obj.Z2; 
           if ~obj.drawn
              obj.drawn = true; 
              obj.P1Handle = plot3(p1(1),p1(2),p1(3),'r*');
              obj.P1_X_AxisHandle = plot3([p1(1) x1(1)],[p1(2) x1(2)],[p1(3) x1(3)],'r');
              obj.P1_Y_AxisHandle = plot3([p1(1) y1(1)],[p1(2) y1(2)],[p1(3) y1(3)],'r');
              obj.P1_Z_AxisHandle = plot3([p1(1) z1(1)],[p1(2) z1(2)],[p1(3) z1(3)],'r');
              obj.P1_Z_pointHandle = plot3(z1(1),z1(2),z1(3),'ro');
              
              obj.P2Handle = plot3(p2(1),p2(2),p2(3),'r*');
              obj.P2_X_AxisHandle = plot3([p2(1) x2(1)],[p2(2) x2(2)],[p2(3) x2(3)],'b');
              obj.P2_Y_AxisHandle = plot3([p2(1) y2(1)],[p2(2) y2(2)],[p2(3) y2(3)],'b');
              obj.P2_Z_AxisHandle = plot3([p2(1) z2(1)],[p2(2) z2(2)],[p2(3) z2(3)],'b');
              obj.P2_Z_pointHandle = plot3(z2(1),z2(2),z2(3),'bo');
              
              obj.labelHandle = text(p1(1),p1(2),p1(3)+0.5,['Joint ' num2str(obj.jointID)]);
           else
              set(obj.P1Handle,'XData',p1(1),'YData',p1(2),'ZData',p1(3)); 
              set(obj.P1_X_AxisHandle,'XData',[p1(1) x1(1)],'YData',[p1(2) x1(2)],'ZData',[p1(3) x1(3)]);
              set(obj.P1_Y_AxisHandle,'XData',[p1(1) y1(1)],'YData',[p1(2) y1(2)],'ZData',[p1(3) y1(3)]);
              set(obj.P1_Z_AxisHandle,'XData',[p1(1) z1(1)],'YData',[p1(2) z1(2)],'ZData',[p1(3) z1(3)]);
              set(obj.P1_Z_pointHandle,'XData',z1(1),'YData',z1(2),'ZData',z1(3)); 
              set(obj.P2Handle,'XData',p2(1),'YData',p2(2),'ZData',p2(3)); 
              set(obj.P2_X_AxisHandle,'XData',[p2(1) x2(1)],'YData',[p2(2) x2(2)],'ZData',[p2(3) x2(3)]);
              set(obj.P2_Y_AxisHandle,'XData',[p2(1) y2(1)],'YData',[p2(2) y2(2)],'ZData',[p2(3) y2(3)]);
              set(obj.P2_Z_AxisHandle,'XData',[p2(1) z2(1)],'YData',[p2(2) z2(2)],'ZData',[p2(3) z2(3)]);
              set(obj.P2_Z_pointHandle,'XData',z2(1),'YData',z2(2),'ZData',z2(3)); 
              set(obj.labelHandle,'Position',p1+[0;0;0.5]); 
           end
       end
       
       % Although we call this "BenderValues" it is quite different from
       % what Bender et al. use...
       function [Jn C Cdot] = getBenderValues( obj )
 
          % Pts = [p z x]
          W_aPts = [obj.P1 obj.Z1 obj.X1];      
          W_bPts = [obj.P2 obj.Z2 obj.X2]; 
          aJn1 = [eye(3,3)   -hat(W_aPts(1:3, 1))]; % p1
          aJn2 = [eye(3,3)   -hat(W_aPts(1:3, 2))]; % z1
          aJn3 = [eye(3,3)   -hat(W_aPts(1:3, 3))]; % x1
          bJn1 = [eye(3,3)   -hat(W_bPts(1:3, 1))]; % p2
          bJn2 = [eye(3,3)   -hat(W_bPts(1:3, 2))]; % z2
          bJn3 = [eye(3,3)   -hat(W_bPts(1:3, 3))]; % x2
          
          Wdiff = W_aPts - W_bPts; 
          if strcmp(obj.type,'prismatic')
              % Prismatic
              Jn = [-aJn1(1:2,:)   bJn1(1:2,:)     % Not sure this is the correct sign
                    -aJn2(1:2,:)   bJn2(1:2,:)
                    -aJn3(2,:)     bJn3(2,:)   ];
          elseif strcmp(obj.type,'revolute')
              % Revolute
              Jn = [aJn1(1:3,:)   -bJn1(1:3,:);
                    aJn2(1:2,:)   -bJn2(1:2,:)];
              C = [Wdiff(1:3,1); Wdiff(1:2,2)]; 
          end
          
          nu = [obj.body1.nu; obj.body2.nu]; 
          Cdot = Jn * nu;
  
       end % End getBenderJacobian()
      
       
       % Returns the mass-inertia matrix for both bodies in the joint
      % 
       function M = getMassInertiaMatrix( obj )
           if obj.body1.static
               M = obj.body2.massInertiaMatrix(); 
           elseif obj.body2.static
               M = obj.body1.massInertiaMatrix();
           else
               M = zeros(12);
               M(1:6,1:6) = obj.body1.massInertiaMatrix();
               M(7:12,7:12) = obj.body2.massInertiaMatrix(); 
           end
       end
       
       % Returns constraint Jacobian and error for joint correction in world coordinates
       function [Gb C Cdot] = getJointCorrectionValues( obj )
           p1 = obj.P1;             p2 = obj.P2;
           x1 = obj.X1-p1;          x2 = obj.X2-p2;
           y1 = obj.Y1-p1;          y2 = obj.Y2-p2;
           z1 = obj.Z1-p1;          z2 = obj.Z2-p2; 
           r1 = p1-obj.body1.u;     r2 = p2-obj.body2.u; 
           r1z = obj.Z1-p1;         r2z = obj.Z2-p2; 
           
                    % DEBUG
                    x2=x1; y2=y1; z2=z1; 
           
           Perr = p1-p2;            % Error of the joint origins
           Zerr = obj.Z1-obj.Z2;    % Error of the Z points
           Xerr = obj.X1-obj.X2;    % Error of the X points
           
           b1 = obj.body1;          b2 = obj.body2; 
           jntType = obj.jntCode;  
           if jntType == 1          % Fixed
                if b1.static, G1 = []; else
                    G1 = [       x1             y1            z1            x1             y1            z1
                           cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) cross3(r1z,x1) cross3(r1z,y1) cross3(r1z,z1) ]; 
                end
                if b2.static, G2 = []; else
                    G2 = [       -x2            -y2            -z2            -x2             -y2             -z2
                           cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) cross3(r2z,-x2) cross3(r2z,-y2) cross3(r2z,-z2) ]; 
                end
                Gb = [G1; G2]; 
                C = [ dot(Perr, x1)  % Constraint error
                      dot(Perr, y1)
                      dot(Perr, z1)
                      dot(Zerr, x1)
                      dot(Zerr, y1)
                      dot(Zerr, z1) ];  
             
           elseif jntType == 2      % Revolute
                if b1.static, G1 = []; else
                    G1 = [       x1             y1            z1            x1             y1    
                           cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) cross3(r1z,x1) cross3(r1z,y1) ]; 
                end
                if b2.static, G2 = []; else
                    G2 = [       -x2            -y2            -z2            -x2             -y2 
                           cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) cross3(r2z,-x2) cross3(r2z,-y2) ]; 
                end
                Gb = [G1; G2]; 
                C = [ dot(Perr, x1)  % Constraint error
                      dot(Perr, y1)
                      dot(Perr, z1)
                      dot(Zerr, x1)
                      dot(Zerr, y1) ];  
                  
           elseif jntType == 3      % Prismatic
                if b1.static, G1 = []; else
                    r1x = obj.X1-p1;  % Prismatic is a special case where we constrain the Joints' X axes in the y direction
                    G1 = [       x1             y1           x1             y1            y1
                           cross3(r1,x1) cross3(r1,y1) cross3(r1z,x1) cross3(r1z,y1) cross3(r1x,y1) ]; 
                end
                if b2.static, G2 = []; else
                    r2x = obj.X2-p2; 
                    G2 = [       -x2            -y2            -x2             -y2             -y2
                           cross3(r2,-x2) cross3(r2,-y2) cross3(r2z,-x2) cross3(r2z,-y2) cross3(r2x,-y2) ]; 
                end
                Gb = [G1; G2]; 
                C = [ dot(Perr, x1)  % Constraint error
                      dot(Perr, y1)
                      dot(Zerr, x1)
                      dot(Zerr, y1)
                      dot(Xerr, y1) ];  

           elseif jntType == 4      % Cylindrical
               if b1.static, G1 = []; else
                    G1 = [       x1             y1           x1             y1      
                           cross3(r1,x1) cross3(r1,y1) cross3(r1z,x1) cross3(r1z,y1) ]; 
                end
                if b2.static, G2 = []; else
                    G2 = [       -x2            -y2            -x2             -y2      
                           cross3(r2,-x2) cross3(r2,-y2) cross3(r2z,-x2) cross3(r2z,-y2) ]; 
                end
                Gb = [G1; G2]; 
                C = [ dot(Perr, x1)  
                      dot(Perr, y1)
                      dot(Zerr, x1)
                      dot(Zerr, y1) ];  
                  
           elseif jntType == 5      % Spherical
               if b1.static, G1 = []; else
                G1 = [       x1             y1            z1             
                       cross3(r1,x1) cross3(r1,y1) cross3(r1,z1) ]; 
                end
                if b2.static, G2 = []; else
                G2 = [       -x2            -y2            -z2     
                       cross3(r2,-x2) cross3(r2,-y2) cross3(r2,-z2) ]; 
                end
                Gb = [G1; G2]; 
                C = [ dot(Perr, x1)  
                      dot(Perr, y1)
                      dot(Perr, z1) ];  
           end

           % Determine Cdot
           if obj.body1.static 
               nu = obj.body2.nu;
           elseif obj.body2.static 
               nu = obj.body1.nu;
           else
               nu = [obj.body1.nu; obj.body2.nu];
           end
           Cdot = Gb' * nu; 
       end
       
       % Get the external forces of the bodies
       function fext = Fext( obj)
          if obj.body1.static 
              fext = obj.body2.Fext 
          elseif obj.body2.static 
              fext = obj.body1.Fext
          else
              fext = [ obj.body1.Fext
                       obj.body2.Fext ]; 
          end
       end
       
       
   end
    
    
end

















