
% Gripper Controller
 
function gripperController( Sim ) 
  JOINT_LIMIT_MIN = 0.0;    % Limits of gripper position
  JOINT_LIMIT_MAX = 0.02;
  
  td = min(JOINT_LIMIT_MAX, Sim.STEP*.0001);
  theta_desired = [td; td];
  
  % Clear applied forces
  for j=1:length(Sim.Joints)
    %Sim.Joints(j).body1.Fext(1:6) = [0;0;0;0;0;0]; 
    Sim.Joints(j).body2.Fext(1:6) = [0;0;0;0;0;0]; 
  end

  % Determine joint torques
  for j=1:length(Sim.Joints)
    J = Sim.Joints(j);
    %disp( J.theta );

    body1 = J.body1;
    body2 = J.body2; 

    Perror = theta_desired(j) - J.theta;  % The proportional error
    Deriv  = (J.theta - J.theta_prev) / Sim.h; 
    joint_frame_force = 10*Perror - 1*Deriv; % P controller

    f1 = J.T1world(1:3,1:3)' * [0;0;  joint_frame_force];  
    body1.Fext(1:3) = body1.Fext(1:3) + f1;

    f2 = J.T2world(1:3,1:3)' * [0;0; -joint_frame_force];
    body2.Fext(1:3) = body2.Fext(1:3) + f2; 

    %disp(['   ' num2str(j) ' Joint error: ' num2str(Perror) ' Deriv: ' num2str(Deriv) ' f1: ' num2str(f1')]);
    %disp(['        Error is ' num2str(theta_desired(j)) ' - ' num2str(J.theta) ]);
  end
   
   % Check joint limits
   % Joint 1
   J = Sim.Joints(1);
   body2 = J.body2;
   if J.theta < JOINT_LIMIT_MIN
      f2 = J.T2world(1:3,1:3) * [0;0; abs(J.theta)];    % Force, not torque; so the point of application matters
      body2.Fext(1:3) = f2;
   elseif J.theta > JOINT_LIMIT_MAX
      f2 = J.T2world(1:3,1:3) * 5*[0;0; JOINT_LIMIT_MAX-J.theta];    % Force, not torque; so the point of application matters
      body2.Fext(1:3) = f2;
   end
   
   J = Sim.Joints(2);
   body2 = J.body2;
   if J.theta < JOINT_LIMIT_MIN
      f2 = J.T2world(1:3,1:3) * [0;0; abs(J.theta)];    % Force, not torque; so the point of application matters
      body2.Fext(1:3) = f2;
   elseif J.theta > JOINT_LIMIT_MAX
      f2 = J.T2world(1:3,1:3) * 5*[0;0; JOINT_LIMIT_MAX-J.theta];    % Force, not torque; so the point of application matters
      body2.Fext(1:3) = f2;
   end

end












