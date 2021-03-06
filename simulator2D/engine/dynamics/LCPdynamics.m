%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function newNU = LCPdynamics( sim )

   M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   if sim.FRICTION
        Gf = sim.dynamics.Gf;
        U = sim.dynamics.U;
        E = sim.dynamics.E;
   end
    
   NU = sim.dynamics.NU;
   FX = sim.dynamics.FX;
  PSI = sim.dynamics.PSI;
   nc = length(sim.contacts);
    h = sim.h;

  %% Construct A and b as LCP
  if sim.FRICTION
      MinvGn = M\Gn;
      MinvGf = M\Gf;

      A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
            Gf'*MinvGn   Gf'*MinvGf  E
            U            -E'         zeros(nc)];

      b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
            Gf'*(NU + M\FX*h);
            zeros(nc,1) ];
      
      % Solve with LEMKE
      z = lemke( A, b, zeros(length(b),1) ); 

      % Calculate updated velocities. 
      % The impulse in the normal direction per active body.
      Pn = z(1:nc);
      % The impulse in the friction directions per active body.
      Pf = z(nc+1:nc + sim.num_fricdirs*nc);
      % Calculate new velocites
      newNU = NU + MinvGn*Pn + MinvGf*Pf + M\FX*h;
  
  else  % Same but without Gf
      MinvGn = M\Gn;

      A = Gn'*MinvGn;

      b = [ Gn'*(NU + M\FX*h) + PSI/h ];    % FX*h could be replaced if we stored Pext instead of Fext
            
      % Solve with LEMKE
      z = lemke( A, b, zeros(length(b),1) ); 

      % Calculate updated velocities. 
      Pn = z(1:nc); % The impulse in the normal direction per active body.
      newNU = NU + MinvGn*Pn + M\FX*h;  % Calculate new velocites
      
  end
  
end



