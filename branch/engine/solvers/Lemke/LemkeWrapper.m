
function LemkeWrapper( sim )

  % Load the dynamics info for recovering the velocity solution.
  M = sim.dynamics.M;
  NU = sim.dynamics.NU;
  FX = sim.dynamics.FX;
  MinvGn = sim.dynamics.MinvGn;
  MinvGf = sim.dynamics.MinvGf; 
  nc = length(sim.Contacts);
  h = sim.h;
  A = sim.dynamics.A;
  b = sim.dynamics.b; 
  mm = length(b);

  % Solve with LEMKE
  [z, iter, err] = lemke( A, ... 
                          b, ...
                          zeros(size(A,1),1) );
                      
  % Handle potential errors 
  if err ~= 0
    if size(err) == 1
        disp(['LCP Error: ' num2str(err)]);
    else
        disp('LCP Error');
    end
  end
  % Calculate updated velocities. Quaternion updates occur in Simulation.m
  % The impulse in the normal direction per active body.
  Pn = z(1:nc);
  % The impulse in the friction directions per active body.
  Pf = z(nc+1:nc + sim.num_fricdirs*nc);
  % Calculate new velocites
  NUnew = NU + MinvGn*Pn + MinvGf*Pf + M\FX*h;

  solver_iteration = iter;
  solver_error = z'*(A*z + b);
  problem_size = mm;
  info = [solver_iteration; solver_error; problem_size];
  sim.z = NUnew;     
  %sim.info = info;
                      