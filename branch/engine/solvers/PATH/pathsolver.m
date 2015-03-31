
% Since PATH uses the wrapper function pathlcp.m, we need to have a wrapper
% for that wrapper that takes the Simulation object and calls pathlcp.m. 

function pathsolver( sim )  
  A = sim.dynamics.A;           % Load objects from obj.dynamics
  b = sim.dynamics.b; 
  nd = sim.num_fricdirs;
  nb = length(sim.activeBodies);
  nc = length(sim.Contacts); 
  nj = length(sim.Joints); 
  njc = sim.num_jointConstraints; 
  ns = sim.num_subContacts; 

  % Solve the MCP  
  problem_size = size(A,1);
  z0 = zeros(problem_size,1);       
  big=10^20;    
  u = big*ones(problem_size,1);
  
  if sim.FRICTION && nd > 0
      l = [-big*ones(6*nb,1);
           zeros((2+nd)*nc+(ns-nc) +5*nj,1)];
  else
      l = [-big*ones(6*nb + njc,1)           % Changed b/c of joints
       zeros(nc+(ns-nc),1)];
  end
  
  [sim.z,~] = pathlcp(A,b,l,u,z0);

  