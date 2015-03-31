%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% preDynamics.m
%
% Generates submatrices that are common for dynamics formulations including
%   M   - Mass-inertia
%   Gn  - Contact normal wrench
%   Gf  - Contact friction wrench
%   Gb  - Bilateral constraint wrench
%   E   - Mask matrix for friction
%   U   - Coefficients of friction
% 
%   NU  - Velocities, including rotational           These 3 are vectors
%   FX  - External forces, including rotational
%   PSI - Gap distances 

function sim = preDynamics( sim )

  % We need to add joint bodies to the set of activeBodies.  Further, by
  % doing this first, we ensure that the mass-inertia matrix contains joint
  % bodies first.  This is useful in postDynamics when applying joint corrections.  
  nj = length(sim.joints); 
  for j=1:nj
     sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
  end
  njc = sim.num_jointConstraints; 

  %% Useful vars 
  nb = sim.num_activeBodies;    % Number of bodies with contacts
  nc = length(sim.contacts);    % Number of contacts
  nd = 2;        % Number of directions in discrete friction "cone"
  

  %% Init submatrices 
  M = sparse(3*nb,3*nb);
  Gn = sparse(3*nb,nc);
  if sim.FRICTION
      Gf = sparse(3*nb,2*nc);  
      U = sparse(nc,nc);
      E = sparse(nd*nc,nc);  % TODO: joint friction
  end
  Gb = sparse(3*nb,njc);
  NU = zeros(3*nb,1);   % Velocity, including angular   
  FX = sparse(3*nb,1);   % External force (not impulse!) 
  PSI = zeros(nc,1);    % Gap distance per contact, psi_n  
  sim.num_subContacts = 0; 
  

  %% Calculate submatrices
  
  % M, NU, and FX
  for b=1:length(sim.bodies)
     B = sim.bodies(b);
     if B.active && B.dynamic
        cID = B.bodyContactID;
        M(3*cID-2:3*cID,3*cID-2:3*cID) = diag([B.mass B.mass B.J]);  
        NU(3*cID-2:3*cID) = B.nu;                  % NU
        FX(3*cID-2:3*cID) = B.Fext;                % FX
     end
  end

  % Gn, E, U, and Gf
  for cID=1:nc
      
    C = sim.contacts(cID);
    PSI(cID) = C.psi_n(1);                                          % PSI

    % For every contact, there are two bodies which enter Gn and Gf 
    % (unless contact was with a static body). 
    
    % Body 1
    B1 = sim.bodies(C.body1_id);
    if B1.dynamic
        r1 = C.p1' - B1.pos;
        body1id = B1.bodyContactID;
        Gn_i1 = [-C.normal'; cross2d(r1,-C.normal)];
        Gn(3*body1id-2:3*body1id,cID) = Gn_i1;
    end
    % Body 2
    B2 = sim.bodies(C.body2_id); 
    if sim.bodies(C.body2_id).dynamic
        r2 = (C.p1+C.normal*C.psi_n)' - B2.pos;
        body2id = B2.bodyContactID;
        Gn_i2 = [C.normal'; cross2d(r2,C.normal)];
        Gn(3*body2id-2:3*body2id,cID) = Gn_i2;
    end
    
    if sim.FRICTION
        % E 
        E(nd*cID-(nd-1):nd*cID,cID) = ones(nd,1);   
        % U
        U(cID,cID) = 0.5*B1.mu * B2.mu;     
        % Gf
        % TODO: Select the initial tangent opposing motion
        d = [-C.normal(2); C.normal(1)];
        if B1.dynamic
            Gf(3*body1id-2:3*body1id, nd*(cID-1)+1) = [d; cross2d(r1,d)];
            Gf(3*body1id-2:3*body1id, nd*(cID-1)+2) = [-d; cross2d(r1,-d)];
        end
        if B2.dynamic
            Gf(3*body2id-2:3*body2id, nd*(cID-1)+1) = [d; cross2d(r2,d)];
            Gf(3*body2id-2:3*body2id, nd*(cID-1)+2) = [-d; cross2d(r2,-d)];
        end
    end
  end
  
  %% Joint Dynamics 
  joint_bn = zeros(njc, 1);
    
  for j=1:nj
     sim = updateJoint( sim, j );
     Jnt = sim.joints(j); 
     b1id = sim.bodies(Jnt.body1id).bodyContactID;
     b2id = sim.bodies(Jnt.body2id).bodyContactID;
     constIndex = Jnt.constraintIndex; 

     [G1c G2c] = joint_Jacobians(sim,j);
     [C Cdot] = joint_constraintError(sim,j);   % TODO: This is probably not necessary.
     %[C Cdot] = Jnt.constraintError();
     %bj = 0.0*C/sim.h + Cdot;    
     bj = 0.9*Cdot;  % Maybe add a coefficient 0<=C<=1 for stability.
     
     if sim.bodies(Jnt.body1id).dynamic
        Gb(3*b1id-2:3*b1id, constIndex) = G1c; 
        %if sim.FRICTION
            %Gf(6*b1id-5:6*b1id, nd*nc+j) = G1f;   % TODO: not yet implemented
        %end
     end
     if sim.bodies(Jnt.body2id).dynamic
        Gb(3*b2id-2:3*b2id, constIndex) = G2c;  % TODO: fix indices 
        %if sim.FRICTION
            %Gf(6*b2id-5:6*b2id, nd*nc+j) = G2f; % TODO: this is wrong now
        %end
     end
     
     if sim.FRICTION
         % TODO: joint friction
     end
     
     % Store joint constraint errors for vector b
     joint_bn(constIndex,1) = bj;  

  end
  sim.dynamics.joint_bn = joint_bn; 
  
  
  %% Store values in the struct sim.dynamics  
  sim.dynamics.M = M;        % It seems that assignment here is faster than
  sim.dynamics.Gn = Gn;      % referencing the struct multiple times above.
  sim.dynamics.Gb = Gb; 
  if sim.FRICTION
      sim.dynamics.Gf = Gf;
      sim.dynamics.U = U;
      sim.dynamics.E = E; 
  end
  sim.dynamics.NU = NU;
  sim.dynamics.FX = FX;
  sim.dynamics.PSI = PSI; 
  
end

