%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% kinematic_update.m
%
% Assumes that the dynamics solution has update obj.z with the active
% bodies' new velocities.  

function kinematic_update( sim )

    h = sim.h; 

    % Apply results from solution, as well as update bodies not in contact. 
    % Some redundant variables at the moment...    
    for i = 1 : length(sim.P)     
       body = sim.P{i};  
       if body.static, continue; end;  % Don't update static bodies. 
       %if body.numJoints > 0, continue; end; % NOTE: we skip over bodies with joints because 
                                             %  they are updated in BenderJointCorrection.m
       
       % Body had no constraints (apply external forces)
       if body.ContactCount <= 0  % &&  body.numJoints <=0  
           body.nu = body.nu + (body.Fext/body.mass)*h;   
           body.u = body.u + h*body.nu(1:3);  % Translation

           w = body.nu(4:6) * h;              % Rotation 
           wn = norm(w); 
           if wn~=0
               body.quat = quatmultiply( body.quat,[-cos(wn/2) (w/wn)'*sin(wn/2)] );
               body.quat = body.quat / norm(body.quat);
           end
           
       % Body had a constraint
       else   
          if strcmp(body.body_type, 'particle')
                bodyID = body.BodyIndex;
                nu = sim.z(3*bodyID-2:3*bodyID); 
                
                % State Updates
                body.nu = nu; 
                body.u = body.u + h*nu(1:3);    % Translation
          else
            bodyID = body.BodyIndex; 
            NUnew = sim.solution.NUnew;
            nu = NUnew(6*bodyID-5:6*bodyID); 
            % State Updates
            body.nu = nu; 
            body.u = body.u + h*nu(1:3);    % Translation
            w = nu(4:6) * h;                        % Rotation 
            wn = norm(w); 
            if wn~= 0
                body.quat = quatmultiply( body.quat,[-cos(wn/2) (w/wn)'*sin(wn/2)] );
                body.quat = body.quat / norm(body.quat);
            end
          end
       end
       %% Update union bodies
       if strcmp(body.body_type, 'union')
           for m = 1:body.num
               body.UNION(m).quat = body.quat;      % Rotation 
               body.UNION(m).u = body.u + ...       % Translation
                   quatrotate(body.quat, body.UNION(m).offset')';
           end
       end
       % Update external forces
       % Kind of hackish since obj_union is the only body_type to
       % currently have appliedForces
       if strcmp(body.body_type, 'union')
            % first apply the controller to control the foot on the
              % terrain balanled itself
%                   value_of_torque = 1000;
%                  
%                   % apply torque to rotate along the y axis if the toe of the foot is higher or lower than the heel; The shoe is along x axis
%                   if body.UNION(3).u(3) > body.UNION(1).u(3)  || body.UNION(6).u(3) > body.UNION(4).u(3) 
%                       height_diff = 0.5*((body.UNION(3).u(3) - body.UNION(1).u(3)) + (body.UNION(6).u(3) - body.UNION(4).u(3)));
%                       body.TuneTorque([0; value_of_torque*height_diff; 0]);
%                   else if body.UNION(3).u(3) < body.UNION(1).u(3)  || body.UNION(6).u(3) < body.UNION(4).u(3) 
%                           height_diff = 0.5*((body.UNION(1).u(3) - body.UNION(3).u(3)) + (body.UNION(4).u(3) - body.UNION(6).u(3)));
%                           body.TuneTorque([0; -value_of_torque*height_diff; 0]);
%                       else
%                           body.tune_torque(2) = 0;
%                       end
%                   end
%                   
%                   % apply torque to rotate along the x axis if one side of the foot is higher or lower than the other;
%                   if body.UNION(5).u(3) > body.UNION(2).u(3)  || body.UNION(4).u(3) > body.UNION(1).u(3) 
%                       height_diff = 0.5*((body.UNION(5).u(3) - body.UNION(2).u(3)) + (body.UNION(4).u(3) - body.UNION(1).u(3)));
%                       body.TuneTorque([value_of_torque * height_diff; 0; 0]);
%                   else if body.UNION(5).u(3) < body.UNION(2).u(3) || body.UNION(4).u(3) < body.UNION(1).u(3) 
%                           height_diff = 0.5*((body.UNION(2).u(3) - body.UNION(5).u(3)) + (body.UNION(1).u(3) - body.UNION(4).u(3)));
%                           body.TuneTorque([-value_of_torque * height_diff; 0; 0]);
%                       else
%                           body.tune_torque(1) = 0;
%                       end
%                   end


           % Initialize Fext
           body.Fext = body.Fext_constant; 
            for f=1:length(body.appliedForces)
               if body.appliedForces(f).t0 <= sim.TIME && body.appliedForces(f).tf >= sim.TIME 

                  % If applying to Center of Mass, simply apply
                  if body.appliedForces(f).p == [0; 0; 0]
                      body.Fext = body.Fext + [body.appliedForces(f).F;0;0;0];  % No torque through COM
                  else 
                      % Put F in local frame
                      Flocal = quatrotate(quatinv(body.quat), body.appliedForces(f).F')'; 

                      % Angle between p and F
                      theta = acos ( ...
                                    dot( Flocal, body.appliedForces(f).p ) / ...
                                    ( norm(Flocal) * norm(body.appliedForces(f).p) ) );

                      % Calculate F and T   
                      r = norm(body.appliedForces(f).p);  % Length of p
                      n = body.appliedForces(f).p/r;      % Normal vector
                      t = cross(body.appliedForces(f).p, body.appliedForces(f).F);  % Tangent vector = p x F
                      % Check if force passes through COM
                      if t == 0
                          Fresult = [Flocal];
                          Tresult = [0;0;0];
                      else 
                          t = t/norm(t); 
                          Fresult = dot(Flocal, n) * n;
                          Tresult = norm(Flocal) * norm(body.appliedForces(f).p) * sin(theta) * t;
                      end
                      % Convert F and T to world frame
                      F = quatrotate( body.quat, Fresult' )';
                      T = quatrotate( body.quat, Tresult' )';
                      T = T + body.tune_torque;
                      % Set Fext
                      body.Fext = body.Fext + [F; T];
                  end
               end
            end
       end

    end

end

