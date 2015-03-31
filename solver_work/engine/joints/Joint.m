%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Joint.m 
%
% Represents a joint between two bodies.  

classdef Joint < handle
   properties (SetAccess='public') 
       jointID      % A Unique ID
       type         % Type of joint
       mask         % A column mask for selecting the joint type
       body1        % Pointer to body1
       body2        % Pointer to body2
       numConstraints % The number of constraints this joint has
       numPreviousConstraints % The total joint constraints for all previous joints 
       
       pos          % Initial world position of joint when initialized
       xdir         % Initial x-direction of joint
       ydir         % Initial y-direction 
       zdir         % Initial z-direction (always considered the AXIS of the joint)
       
       linkedJoints % List of jointIDs of joints that are in a chain with this joint
       
       mu           % Joint friction
       
       quat1        % Quaternion orientation of joint frame from body1 to joint
       jointPos1    % Position of joint frame from body1 COM to joint
       T1           % Transformation matrix from body1 COM to joint on body1
       
       quat2        % Quaternion orientation of joint frame from body1 to joint
       jointPos2    % Position of joint frame from body1 COM to joint
       T2           % Transformation matrix from body2 COM to joint on body2
       
       p1           % Position of joint as body1 sees it, in WORLD coordinates
       x1           % Position of X=1 on body1's joint frame, in WORLD coordinates
       y1           % Position of Y=1
       z1           % Position of Z=1
       V1           % A velocity term of points p1 and z1 in world coordinates
       
       p2           % Position of joint as body2 sees it, in WORLD coordinates
       x2           % Position of X=1 on body2's joint frame, in WORLD coordinates
       y2           % Position of Y=1
       z2           % Position of Z=1
       V2           % A velocity term of points p2 and z2 in WORLD coordinates
              
       % Graphics data
       drawn
       p1Handle
       p1_X_AxisHandle
       p1_Y_AxisHandle
       p1_Z_AxisHandle
       p2Handle
       p2_X_AxisHandle
       p2_Y_AxisHandle
       p2_Z_AxisHandle
       labelHandle
       
       
       % For joint corrections
       bender       % A struct for storing bender data
   end
   
   methods
       %% Constructor
       function obj = Joint( type, body1, pos1, quat1, body2, pos2, quat2, npc)
           obj.type = type;         
           
           obj.body1 = body1;
           obj.jointPos1 = pos1;
           obj.quat1 = quat1;
           obj.body2 = body2;
           obj.jointPos2 = pos2;
           obj.quat2 = quat2; 
           obj.initTransformations(); 
           
           obj.update(); 
           obj.pos = obj.p1;  % Since bodies have not yet moved, p1 = p2
           obj.xdir = obj.x1 - obj.p1;
           obj.ydir = obj.y1 - obj.p1;
           obj.zdir = obj.z1 - obj.p1; 
           
           obj.linkedJoints = []; 
           obj.mu = body1.mu * body2.mu; 
           obj.drawn = false; 
           
           % Mask columns to obtain specific joint types
           if strcmp(obj.type, 'fixed')
               obj.mask = [1 2 3 4 5 6];
           elseif strcmp(obj.type, 'revolute')
               obj.mask = [1 2 3 4 5  ];
           elseif strcmp(obj.type, 'prismatic')
               obj.mask = [1 2   4 5 6];
           elseif strcmp(obj.type, 'spherical')
               obj.mask = [1 2 3      ];
           elseif strcmp(obj.type, 'cylindrical')
               obj.mask = [1 2   4 5  ];
           end
           obj.numPreviousConstraints = npc; 
           obj.numConstraints = length(obj.mask); 
       end
       
       %% initTransformations()
       function obj = initTransformations( obj )
           obj.T1 = zeros(4,4);
           obj.T1(4,4) = 1;
           obj.T1(1:3,1:3) = quat2rot(obj.quat1);
           obj.T1(1:3,4) = obj.jointPos1; 
           
           obj.T2 = zeros(4,4); 
           obj.T2(4,4) = 1;
           obj.T2(1:3,1:3) = quat2rot(obj.quat2);
           obj.T2(1:3,4) = obj.jointPos2;
       end
       
       %% update() 
       % Updates values affected by motion of bodies 1 & 2
       function obj = update( obj )
           T = zeros(4,4); 
           T(4,4) = 1;
           T(1:3,1:3) = quat2rot(obj.body1.quat);
           T(1:3,4) = obj.body1.u; 
           T = T*obj.T1;
           obj.p1 = T(1:3,4);
           obj.x1 = T*[1;0;0;1]; obj.x1 = obj.x1(1:3);
           obj.y1 = T*[0;1;0;1]; obj.y1 = obj.y1(1:3);
           obj.z1 = T*[0;0;1;1]; obj.z1 = obj.z1(1:3);
           
           T = zeros(4,4); 
           T(4,4) = 1;
           T(1:3,1:3) = quat2rot(obj.body2.quat);
           T(1:3,4) = obj.body2.u; 
           T = T*obj.T2;
           obj.p2 = T(1:3,4);
           obj.x2 = T*[1;0;0;1]; obj.x2 = obj.x2(1:3);
           obj.y2 = T*[0;1;0;1]; obj.y2 = obj.y2(1:3);
           obj.z2 = T*[0;0;1;1]; obj.z2 = obj.z2(1:3);
           
           if obj.body1.static
               obj.V1 = zeros(6,1); 
           else
               obj.V1 = [ obj.body1.nu(1:3) + cross(obj.body1.nu(4:6), obj.p1 - obj.body1.u)
                          obj.body1.nu(1:3) + cross(obj.body1.nu(4:6), obj.z1 - obj.body1.u) ];
           end
           
           if obj.body2.static
               obj.V2 = zeros(6,1); 
           else
               obj.V2 = [ obj.body2.nu(1:3) + cross(obj.body2.nu(4:6), obj.p2 - obj.body2.u)
                          obj.body2.nu(1:3) + cross(obj.body2.nu(4:6), obj.z2 - obj.body2.u) ];
           end
           
       end
       
       
       %% The joint constraint Jacobians (Gc, or Gn in the dynamics)
       % Returns the constraint jacobian, broken into body1 and body2 
       function [G1c G2c] = Jacobians( obj )
           
           p1 = obj.p1; 
           x1 = obj.x1 - p1;
           y1 = obj.y1 - p1;
           z1 = obj.z1 - p1;
           r1 = p1 - obj.body1.u;
           
           p2 = obj.p2;
           x2 = obj.x2 - p2;
           y2 = obj.y2 - p2;
           z2 = obj.z2 - p2;
           r2 = p2 - obj.body2.u; 

           zrs = zeros(3,1); 
           
           % Not compuationally efficient, but let's write the whole thing
           if obj.body1.static
               G1c = zeros(6);
           else
               G1c = [      x1              y1            z1       zrs   zrs   zrs 
                      cross(r1,x1)   cross(r1,y1)   cross(r1,z1)   x1    y1    z1  ];
           end
           if obj.body2.static
               G2c = zeros(6);
           else
               G2c = [     -x2             -y2             -z2        zrs    zrs    zrs 
                      cross(r2,-x2)   cross(r2,-y2)   cross(r2,-z2)   -x2    -y2    -z2  ];
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
           C = [obj.p1-obj.p2; cross(obj.z1-obj.p1,obj.z2-obj.p2)];  % TODO: I don't think the second half is correct
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
           
           p1 = obj.p1; 
           x1 = obj.x1;
           y1 = obj.y1;
           z1 = obj.z1; 
           p2 = obj.p2;
           x2 = obj.x2;
           y2 = obj.y2;
           z2 = obj.z2; 
           if ~obj.drawn
              obj.drawn = true; 
              obj.p1Handle = plot3(p1(1),p1(2),p1(3),'r*');
              obj.p1_X_AxisHandle = plot3([p1(1) x1(1)],[p1(2) x1(2)],[p1(3) x1(3)],'r');
              obj.p1_Y_AxisHandle = plot3([p1(1) y1(1)],[p1(2) y1(2)],[p1(3) y1(3)],'r');
              obj.p1_Z_AxisHandle = plot3([p1(1) z1(1)],[p1(2) z1(2)],[p1(3) z1(3)],'r');
              
              obj.p2Handle = plot3(p2(1),p2(2),p2(3),'r*');
              obj.p2_X_AxisHandle = plot3([p2(1) x2(1)],[p2(2) x2(2)],[p2(3) x2(3)],'b');
              obj.p2_Y_AxisHandle = plot3([p2(1) y2(1)],[p2(2) y2(2)],[p2(3) y2(3)],'b');
              obj.p2_Z_AxisHandle = plot3([p2(1) z2(1)],[p2(2) z2(2)],[p2(3) z2(3)],'b');
              
              obj.labelHandle = text(p1(1),p1(2),p1(3)+0.5,['Joint ' num2str(obj.jointID)]);
           else
              set(obj.p1Handle,'XData',p1(1),'YData',p1(2),'ZData',p1(3)); 
              set(obj.p1_X_AxisHandle,'XData',[p1(1) x1(1)],'YData',[p1(2) x1(2)],'ZData',[p1(3) x1(3)]);
              set(obj.p1_Y_AxisHandle,'XData',[p1(1) y1(1)],'YData',[p1(2) y1(2)],'ZData',[p1(3) y1(3)]);
              set(obj.p1_Z_AxisHandle,'XData',[p1(1) z1(1)],'YData',[p1(2) z1(2)],'ZData',[p1(3) z1(3)]);
              set(obj.p2Handle,'XData',p2(1),'YData',p2(2),'ZData',p2(3)); 
              set(obj.p2_X_AxisHandle,'XData',[p2(1) x2(1)],'YData',[p2(2) x2(2)],'ZData',[p2(3) x2(3)]);
              set(obj.p2_Y_AxisHandle,'XData',[p2(1) y2(1)],'YData',[p2(2) y2(2)],'ZData',[p2(3) y2(3)]);
              set(obj.p2_Z_AxisHandle,'XData',[p2(1) z2(1)],'YData',[p2(2) z2(2)],'ZData',[p2(3) z2(3)]);
              set(obj.labelHandle,'Position',p1+[0;0;0.5]); 
           end
       end
       
       % Although we call this "BenderValues" it is quite different from
       % what Bender et al. use...
       function [Jn C Cdot] = getBenderValues( obj )
 
          % Pts = [p z x]
          W_aPts = [obj.p1 obj.z1 obj.x1];      
          W_bPts = [obj.p2 obj.z2 obj.x2]; 
          aJn1 = [eye(3,3)   -hat(W_aPts(1:3, 1))]; % p1
          aJn2 = [eye(3,3)   -hat(W_aPts(1:3, 2))]; % z1
          aJn3 = [eye(3,3)   -hat(W_aPts(1:3, 3))]; % x1
          bJn1 = [eye(3,3)   -hat(W_bPts(1:3, 1))]; % p2
          bJn2 = [eye(3,3)   -hat(W_bPts(1:3, 2))]; % z2
          bJn3 = [eye(3,3)   -hat(W_bPts(1:3, 3))]; % x2
          
          Wdiff = W_aPts - W_bPts; 
          if strcmp(obj.type,'prismatic')
              % Prismatic
              Jn = [-aJn1(1:2,:)   bJn1(1:2,:);     % Not sure this is the correct sign
                    -aJn2(1:2,:)   bJn2(1:2,:);
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
       
   end
    
    
end

