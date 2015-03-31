%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% CDAdynamics.m
%
% Formulate CDA as an MCP and return updated velocities
% 

function CDAdynamics( sim )

%% Useful vars 
  nb = length(sim.activeBodies);
  nc = length(sim.Contacts); 
  ns = sim.num_subContacts;
  nd = sim.num_fricdirs;   

%% Init submatrices 
  M = sim.dynamics.M;
  Gn = sim.dynamics.Gn;
  if (sim.FRICTION)
    Gf = sim.dynamics.Gf;
    U = sim.dynamics.U;
    E = sim.dynamics.E;
  else
    nd = 0;
  end
  
  GaT = zeros(ns-nc, 6*nb);  % Note that this is GaT, not Ga
  E1 = zeros(nc, ns-nc);
  E2 = zeros(ns-nc, ns-nc);
  if nd > 0 
    b = zeros(6*nb+(2+nd)*nc+(ns-nc),1);
  else
    b = zeros(6*nb+nc+(ns-nc),1);
  end

%% Formulate submatrices

  % Gat, E1, E2, and b
  scID = 0;
  idx = 1;
  for i=1:nc         %   TODO: If the the greatest gap is -eps, is that ok? 
    C = sim.Contacts(i);
    nsj = length(C.psi_n); 
    
    r2 = C.p2;
    body1id = sim.P{C.body_1}.BodyIndex;
    
    % Setup b (since we're already looping)
    b(6*nb+C.id) = C.psi_n(1) / sim.h;   % b is completed after A

    Gn_i2 = zeros(6,nsj);
    for sc=1:nsj
        Gn_i2(:,sc) = [C.normal(:,sc); cross3(r2(:,sc), C.normal(:,sc))];  
    end
    
    % Subcontact                            % TODO: This could be easily vectorized
    for sc=1:length(C.psi_n) 
        scID = scID + 1; % Subcontact ID
        
        if sim.P{C.body_1}.static == 0
            if sc==1
                Gn1T = Gn_i2(:,sc)';
            else
                % Ga
                GaT(scID-C.id,6*body1id-5:6*body1id) = Gn1T-Gn_i2(:,sc)';  
            end
        end
    end
    
    
    if nsj > 1      % Both dynamic and static bodies contribute to E1 & E2.
        E1(C.id , idx:idx+nsj-2) = 1;                               % E1
        E2(idx:idx+nsj-2, idx:idx+nsj-2) = tril(ones( nsj-1 ));     % E2
        
        % CDA portion of b:
        bdex = 6*nb + nc + nd*nc + idx;
        if sc > 1
            b(bdex : bdex+nsj-2) = (C.psi_n(1)-C.psi_n(2:nsj)) / sim.h;
        end
        
        idx = idx + nsj - 1;
    end
    
  end

%% Construct A and finish b
if nd > 0 
  % Note: first row is negated
  A = [-M               +Gn                      +Gf         zeros(6*nb,(ns-nc)+nc)   
       Gn'              zeros(nc,nc+nd*nc)       E1          zeros(nc,nc)
       Gf'              zeros(nd*nc,nc+nd*nc+(ns-nc))        E
       GaT              zeros(ns-nc,nc+nd*nc)    E2          zeros((ns-nc),nc)
       zeros(nc,6*nb)   U                        -E'         zeros(nc,(ns-nc)+nc) ];
else % The case where friction is not included (No, not the most efficient).
  A = [-M       Gn                zeros(6*nb,(ns-nc))   
       Gn'      zeros(nc,nc)      E1              
       GaT      zeros(ns-nc,nc)   E2                    ];
end
   
  for i=1:nb          % b was started above, it's finished here.
    j = sim.activeBodies(i);
    low = 6*i-5; high =6*i;
    b(low:high) = M(low:high,low:high) * sim.P{j}.nu + sim.h*sim.P{j}.Fext; 
  end
  
  % Store values for the solver to use
  sim.dynamics.A = A;
  sim.dynamics.b = b; 
  
end


    
