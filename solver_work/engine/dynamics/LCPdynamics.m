%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function LCPdynamics( sim )

    M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   Gf = sim.dynamics.Gf;
    U = sim.dynamics.U;
    E = sim.dynamics.E;
   NU = sim.bodies.velocities;
   FX = sim.bodies.forces;
  PSI = sim.contacts.PSI;
   nc = length(sim.Contacts);
    h = sim.h;

  %% Construct A and b as LCP
  MinvGn = M\Gn;
  MinvGf = M\Gf;

  A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
        Gf'*MinvGn   Gf'*MinvGf  E
        U            -E'         zeros(nc)];

  b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
        Gf'*(NU + M\FX*h);
        zeros(nc,1) ];
  
  % Store values for the solver
  sim.dynamics.A_LCP = A;
  sim.dynamics.b_LCP = b;
  sim.dynamics.z0_LCP = zeros(length(b), 1);
  sim.dynamics.MinvGn = MinvGn;
  sim.dynamics.MinvGf = MinvGf; 
end



