%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% BenderJointCorrection.m
%
% This method is meant to apply Bender's corrections as described in 
% "An impulse-based dynamic simulation system for VR applications" by
%  Jan Bender, Dieter Finkenzeller, and Alfred Schmitt, VC 2005.   
%
% This method is called AFTER a solution is found and BEFORE kinematic
% updates are applied.  So all bodies in sim.P have their information from
% time t0 unchanged, and the results to be applied from dynamics solution 
% are available in sim.z.  

function BenderJointCorrection( sim )   

  epsPos = 10^-5;                   % Joint position error epsilon  
  epsVel = 10^-2;                   % Joint velocity error epsilon 
  maxIters = 5;            
  nj = length(sim.Joints);          % Number of joints 
  nb = sim.num_activeJointBodies;   % Number of bodies involved
  njc = sim.num_jointConstraints;   % Number of joint constraints
  h = sim.h;                        % Simulation time-step 
  
  JointPosError = zeros(nj,1);
  JointVelError = zeros(nj,1); 

  %% For every joint, record values BEFORE update is applied, i.e. at time t0
  for j=1:nj
     J = sim.Joints(j);
     [Gb C Cdot] = J.getBenderValues();
     J.bender.Gb_t0 = Gb; 
     J.update(); 
  end
          
  Piters = 0;
  Viters = 0;
  
  %% Bender position correction 
  for Pcorrection = 1:6
      for j=1:nj
        J = sim.Joints(j); 
        J.update(); 

        b1 = J.body1;
        b2 = J.body2; 

        M = J.getMassInertiaMatrix(); 
        Gb_t0 = J.bender.Gb_t0; 

        [Gb_t1 C_t1 Cdot_t1] = J.getBenderValues(); 
        while norm(C_t1) > 10^-6
            Piters = Piters + 1;

            J.update(); 
            [Gb_t1 C_t1 Cdot_t1] = J.getBenderValues(); Gb_t1     %disp([num2str(norm(C_t1))]);
            dp = -inv(Gb_t0 * inv(M) * Gb_t0') * C_t1/h; 
            dnu = inv(M) * Gb_t0' * dp;

            [wRa, Ba] = ep2rot(b1.quat);
            [wRb, Bb] = ep2rot(b2.quat);

            % Update positions
            b1.u = b1.u + dnu(1:3) * h; 
            b2.u = b2.u + dnu(7:9) * h;

            b1.quat = b1.quat - (Ba * wRa' * dnu(4:6) * h)';        % TODO: the negative is only required because ep2rot uses right
            b2.quat = b2.quat - (Bb * wRb' * dnu(10:12) * h)';      % handed quat, and MATLAB is silly and uses left-handed 
                                                                    % TODO:
                                                                    % also, Ba*wRa' is constant, and doesn't need to
                                                                    % be computed more than once 
            b1.quat = b1.quat / norm(b1.quat);
            b2.quat = b2.quat / norm(b2.quat); 

            % Update velocities
            b1.nu = b1.nu + dnu(1:6);
            b2.nu = b2.nu + dnu(7:12); 
        end

      end
  end 
  
  % Update Jacobian values between position and velocity correction
  for j=1:nj
     J = sim.Joints(j);
     J.update();
     [Gb_t1 C_t1 Cdot_t1] = J.getBenderValues();  
     J.bender.Gb_t1 = Gb_t1; 
  end
  
  %% Bender velocity correction 
  for Vcorrection = 1:6
     for j=1:nj
        J = sim.Joints(j); 
        J.update(); 
        b1 = J.body1;
        b2 = J.body2; 
        
        M = J.getMassInertiaMatrix(); 
        [Gb_t1 C_t1 Cdot_t1] = J.getBenderValues(); 
        Gb_t1 = J.bender.Gb_t1; 
        
        while norm(Cdot_t1) > 10^-6
            Viters = Viters+1;
            J.update(); 
            [Gb_t1 C_t1 Cdot_t1] = J.getBenderValues();     %disp(['Cdot: ' num2str(norm(Cdot_t1))]);
            dp = -inv(Gb_t1 * inv(M) * Gb_t1') * Cdot_t1; 
            dnu = inv(M) * Gb_t1' * dp;

            [wRa, Ba] = ep2rot(b1.quat);
            [wRb, Bb] = ep2rot(b2.quat);

            % Update velocities
            b1.nu = b1.nu + dnu(1:6);
            b2.nu = b2.nu + dnu(7:12); 
        end
     end
  end
  
  disp(['Piters: ' num2str(Piters) ',  Viters: ' num2str(Viters)]);

end % END FUNCTION









