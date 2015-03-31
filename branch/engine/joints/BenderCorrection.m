


function BenderCorrection( sim )

  epsPos = 10^-5;                   % Joint position error epsilon  
  epsVel = 10^-2;                   % Joint velocity error epsilon 
  maxIters = 5;            
  nj = length(sim.Joints);          % Number of joints 
  nb = sim.num_activeJointBodies;   % Number of bodies involved
  nc = sim.num_jointConstraints;  
  h = sim.h;   

  % Initialize the joint variables from the dynamics solution
  nu_lp1 = sim.z(1:6*nb);   % Velocites at time t0+h or step "l+1"
  lam_lp1 = sim.z(6*nb+1 : 6*nb+nc) / sim.h;     % Lamda (impulses) at time t0+h


end

