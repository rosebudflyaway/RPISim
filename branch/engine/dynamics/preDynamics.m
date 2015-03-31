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
%   NU  - Velocities, including rotational            These 3 are vectors
%   FX  - External forces, including rotational
%   PSI - Gap distances 

% There are TWO important parts of this code:  
%   - The first calls a user-defined conroller (if it exists) in order to 
%       generate forces/torques
%   - The second is part forms basic dynamics structures to be used by
%       dynamics formulations in the next stage of the simulator.  
function preDynamics( sim )

  
  %% 
  if sim.hController
     feval(sim.hController, sim); 
  end


  % We need to add joint bodies to the set of activeBodies.  Further, by
  % doing this first, we ensure that the mass-inertia matrix contains joint
  % bodies first.  This is useful in postDynamics when applying joint corrections.  
  nj = length(sim.Joints); 
  for j=1:nj
     sim.addActiveBody(sim.Joints(j).body1.bodyID);
     sim.addActiveBody(sim.Joints(j).body2.bodyID);
  end
  njc = sim.num_jointConstraints; 

  %% Useful vars 
  nb = length(sim.activeBodies); % Number of bodies with contacts
  nc = length(sim.Contacts);   % Number of contacts
  nd = sim.num_fricdirs;           % Number of directions in discrete friction "cone"
  

  %% Init submatrices 
  M = zeros(6*nb);
  Gn = zeros(6*nb,nc);
  if sim.FRICTION
      Gf = zeros(6*nb,nd*nc);  
      U = eye(nc);
      E = zeros(nd*nc,nc);  % TODO: joint friction
  end
  Gb = zeros(6*nb,njc);
  NU = zeros(6*nb,1);   % Velocity, including angular   
  FX = zeros(6*nb,1);   % External force (not impulse!) 
  PSI = zeros(nc,1);  % Gap distance per contact, psi_n  
  sim.num_subContacts = 0; 
  

  %% Calculate submatrices
  % M, NU, and FX
  for i=1:nb
    M(6*i-5:6*i,6*i-5:6*i) = sim.P{sim.activeBodies(i)}.massInertiaMatrix();  
    NU(6*i-5:6*i) = sim.P{sim.activeBodies(i)}.nu;                  % NU
    FX(6*i-5:6*i) = sim.P{sim.activeBodies(i)}.Fext;                % FX
  end
   
  % Gn, E, U, and Gf
  for i=1:nc
      
    C = sim.Contacts(i);
    cID = C.id;
    sim.num_subContacts = sim.num_subContacts + length(C.psi_n);    % TODO: count the number of subcontacts
    
    PSI(cID) = C.psi_n(1);                                          % PSI

    % For every contact, there are two bodies which enter Gn and Gf 
    % (unless contact was with a static body). 
    
    % Body 1
    if sim.P{C.body_1}.static == 0
        r1 = C.p1;
        body1id = sim.P{C.body_1}.BodyIndex;
        Gn_i1 = [-C.normal(:,1); cross3(r1,-C.normal(:,1))];
        Gn(6*body1id-5:6*body1id,cID) = Gn_i1;
    end
    % Body 2
    if sim.P{C.body_2}.static == 0
        r2 = C.p2(:,1);
        body2id = sim.P{C.body_2}.BodyIndex;
        Gn_i2 = [C.normal(:,1); cross3(r2,C.normal(:,1))];
        Gn(6*body2id-5:6*body2id,cID) = Gn_i2;
    end
    
    if sim.FRICTION
        % E 
        E(nd*i-(nd-1):nd*i,i) = ones(nd,1);   
        % U
        U(cID,cID) = 0.5*(sim.P{C.body_1}.mu * sim.P{C.body_2}.mu);     
        % Gf
        if (strcmp(sim.SOLVER, 'mncp_fixed_point'))
            % The following update should not include static bodies.
            if sim.P{C.body_1}.static == 0
                Gf_i1 = zeros(6,2);
                Gf_i1(1:3, 1) = arbitraryTangent(-C.normal(:,1));      % <----
                Gf_i1(1:3, 2) = cross(-C.normal(:,1), Gf_i1(1:3, 1));
                Gf_i1(4:6, 1:2) = [cross(r1, Gf_i1(1:3, 1)), cross(r1, Gf_i1(1:3, 2))];
                Gf(6*body1id-5:6*body1id,2*cID-1:2*cID) = Gf_i1;
            end

            if sim.P{C.body_2}.static == 0
                Gf_i2 = zeros(6,2);
                Gf_i2(1:3, 1) = arbitraryTangent(C.normal(:,1));       % <---- Should this not be the same as for body_1?
                Gf_i2(1:3, 2) = cross(-C.normal(:,1), Gf_i2(1:3, 1));
                Gf_i2(4:6, 1:2) = [cross(r2, Gf_i2(1:3.,1)), cross(r2, Gf_i2(1:3, 2))];
                Gf(6*body2id-5:6*body2id,2*cID-1:2*cID) = Gf_i2;
            end
        else
            for j=1:nd
                d = rot(C.normal(:,1),((j-1)/nd)*(2*pi)) * C.tangent(:,1);    % Friction direction d
                if sim.P{C.body_1}.static == 0
                    Gf(6*body1id-5:6*body1id, nd*(cID-1)+j) = [d; cross3(r1,d)];
                end
                if sim.P{C.body_2}.static == 0
                    Gf(6*body2id-5:6*body2id, nd*(cID-1)+j) = [d; cross3(r2,d)];
                end
            end
        end
    end
  end
  
  %% Joint Dynamics 
  joint_bn = zeros(sim.num_jointConstraints, 1);
    
  for j=1:nj
     Jnt = sim.Joints(j); 
     Jnt.update();                      % TODO: this is currently a redundant call
     b1id = Jnt.body1.BodyIndex;
     b2id = Jnt.body2.BodyIndex; 
     constIndex = Jnt.constraintIndex; 

     [G1c G2c] = Jnt.Jacobians();
     [C Cdot] = Jnt.constraintError();
     bj = 0*C/sim.h + 0.5*Cdot;    
     
     if ~Jnt.body1.static
        Gb(6*b1id-5:6*b1id, constIndex) = G1c; 
        if sim.FRICTION
            Gf(6*b1id-5:6*b1id, nd*nc+j) = G1f;   % TODO: not yet implemented
        end
     end
     if ~Jnt.body2.static
        Gb(6*b2id-5:6*b2id, constIndex) = G2c;  % TODO: fix indices 
        if sim.FRICTION
            Gf(6*b2id-5:6*b2id, nd*nc+j) = G2f; % TODO: this is wrong now
        end
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

