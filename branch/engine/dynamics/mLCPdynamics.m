%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mLCPdynamics.m
%
% Constructs matrices A and b, representing the mixed linear 
% complementarity problem (mLCP) given the current contacts.  

function mLCPdynamics( sim )
  M = sim.dynamics.M;             % Mass-inertia matrix
  Gn = sim.dynamics.Gn;   
  Gb = sim.dynamics.Gb; 
  if sim.FRICTION
     Gf = sim.dynamics.Gf;
     U = sim.dynamics.U;
     E = sim.dynamics.E; 
  end
  NU = sim.dynamics.NU;           % Vector of velocities, including rotational
  FX = sim.dynamics.FX;           % Vector of external forces, including rotational
  PSI = sim.dynamics.PSI;         % Vector of gap distances 
  h = sim.h;                      % Simulation time-step 
  
  nc = length(sim.Contacts); 
  nb = length(sim.activeBodies);
  nj = length(sim.Joints);          % Number of joints
  njc = sim.num_jointConstraints; 
  nd = sim.num_fricdirs; 
  
  % Construct A and b
  % With friction
  if sim.FRICTION
      A = [ -M               Gb                 Gn           Gf          zeros(6*nb,nc+nj)  % Note: first line is negated
            Gn'              zeros(nc,njc)      zeros(nc,nc*(2+nd))
            Gf'              zeros(nd*nc,njc)   zeros(nd*nc,(1+nd)*nc)   E
            zeros(nc,6*nb)   zeros(nc,njc)      U           -E'          zeros(nc+nj) ];   

      b = [ M*NU + FX*h                                                
            PSI / h
            zeros((nd+1)*nc+nj,1) ];
  % No friction
  else
      A = [ -M    Gb    Gn   
            Gb'   zeros(njc,njc+nc)
            Gn'   zeros( nc,njc+nc) ]; 

      b = [ M*NU + FX*h      
            sim.dynamics.joint_bn 
            PSI / h                 ];
  end
 
  % Store formulation, to be passed to solver.
  sim.dynamics.A = A;
  sim.dynamics.b = b;
end 




