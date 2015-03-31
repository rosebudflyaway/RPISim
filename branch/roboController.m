
% This is a test function for controller the joint space of a simple 
% robot in the RmS.  

% roboController
% TODO: have this read from a experiment file in order to determine the
% target joint angles.  
function roboController( Sim ) 

  %theta_desired = [ -Sim.STEP*.1; -Sim.STEP*.1] / (180/pi);
  theta_desired(length(Sim.Joints))=0;
  flag = 1;
  if (Sim.STEP>200)
      flag = 200/Sim.STEP;
  end
  for j=1:length(Sim.Joints)
     Sim.Joints(j).body1.Fext(4:6) = [0;0;0]; 
     Sim.Joints(j).body2.Fext(4:6) = [0;0;0];
     if (Sim.jointAngle() ~= 0)
         theta_desired(j) = (Sim.jointAngle(j)/200)*Sim.STEP*flag/(180/pi);    %it is desinged to complete the simulation in 200 time steps
     else
         theta_desired(j) = [ -Sim.STEP*.1; -Sim.STEP*.1] / (180/pi);
     end
  end

  Ps = [];
  Ds = []; 
  
  for j=1:length(Sim.Joints)
       J = Sim.Joints(j);
       
       body1 = J.body1;
       body2 = J.body2; 

       Perror = theta_desired(j) - J.theta;  % The proportional error
       Deriv  = (J.theta - J.theta_prev); % / Sim.h; 
       joint_frame_torque = 10*Perror - 0.5*Deriv; % PD controller
       
       t1 = J.T1world(1:3,1:3) * [0;0; -joint_frame_torque];  
       body1.Fext(4:6) = body1.Fext(4:6) + t1;
       
       t2 = J.T2world(1:3,1:3) * [0;0; joint_frame_torque];
       body2.Fext(4:6) = body2.Fext(4:6) + t2; 
       
%        disp(['   ' num2str(j) ' Joint error: ' num2str(Perror) ' Deriv: ' num2str(Deriv) ]);
%        disp(['        Error is ' num2str(theta_desired(j)) ' - ' num2str(J.theta) ' = ' num2str(Perror) ]);
%        disp(['              t1: ' num2str(t1')]);
       
        % Let's track joint error
        if Sim.STEP == 1
            J.bender.PERR = Perror;
            J.bender.DERR = Deriv;
        else
            J.bender.PERR = [J.bender.PERR; Perror];
            J.bender.DERR = [J.bender.DERR; Deriv];
        end
        
        Ps = [Ps J.bender.PERR]; 

       % For now, we need to keep track of this in the controller. 
       J.theta_prev = J.theta; 
  end
   
  %figure(2);
  %subplot(2,1,1);  
  %plot(Ps); 
  %title('Joint Error');
  %xlabel('Simulation Step'); ylabel('Error in radians'); 
  %grid on; 
  
  %subplot(2,1,2);  title('Joint Derivative');
  %plot();


end

