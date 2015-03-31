%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% PEGdynamics.m
%
% Formulate CDA as an MCP and return updated velocities
%

% PEGdynamics assumes a particular set of variables exists, namely
%   sim.contacts    % A struct array of Contact
%   sim.Xcontacts   % A struct array of crossContact
%
function newNU = PEGdynamics( sim )


    %% Execute heuristic on contacts
    for i=1:length(sim.contacts)
        if length(sim.contacts(i).psi_n) > 1
            minPsi = inf; %sim.contacts(i).psi_n(1);
            minDex = 1;
            newPsi = false; 
            for j=1:length(sim.contacts(i).psi_n)
                if sim.contacts(i).psi_n(j) > -10^-2 && sim.contacts(i).psi_n(j) < minPsi
                   minPsi = sim.contacts(i).psi_n(j);
                   minDex = j;
                   newPsi = true; 
                end
            end
            if newPsi
               tempPsi = sim.contacts(i).psi_n(1); 
               tempNorm = sim.contacts(i).normal(1,:);
               
               sim.contacts(i).psi_n(1) = sim.contacts(i).psi_n(minDex);
               sim.contacts(i).normal(1,:) = sim.contacts(i).normal(minDex,:);
               
               sim.contacts(i).psi_n(minDex) = tempPsi;
               sim.contacts(i).normal(minDex,:) = tempNorm;
            end
        end
    end


    %% Useful vars
    nb = sim.num_activeBodies;    % Number of bodies with contacts
    nc = length(sim.contacts);    % Number of contacts
    
    if sim.FRICTION
        nd = sim.num_fricdirs;        % Number of directions in discrete friction "cone"
    else
        nd = 0;
    end
        
    ns = length(sim.contacts);
    for i=1:length(sim.contacts)
       ns = ns + length(sim.contacts(i).psi_n)-1; 
    end


    %% Init submatrices
    M = sparse(3*nb,3*nb);
    Gn = sparse(3*nb,nc);
    if sim.FRICTION
        Gf = sparse(3*nb,2*nc);
        U = sparse(nc,nc);
        E = sparse(nd*nc,nc);  % TODO: joint friction
    end
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
            Gn_i1 = [-C.normal(1,:)'; cross2d(r1,-C.normal(1,:))];
            Gn(3*body1id-2:3*body1id,cID) = Gn_i1;
        end
        % Body 2
        B2 = sim.bodies(C.body2_id);
        if sim.bodies(C.body2_id).dynamic
            r2 = (C.p1+C.normal(1,:)*C.psi_n(1))' - B2.pos;
            body2id = B2.bodyContactID;
            Gn_i2 = [C.normal(1,:)'; cross2d(r2,C.normal(1,:))];
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


    %% Init submatrices
    GaT = zeros(ns-nc, 3*nb);  % Note that this is GaT, not Ga
    E1 = zeros(nc, ns-nc);
    E2 = zeros(ns-nc, ns-nc);
    if nd > 0
        b = zeros(3*nb+(2+nd)*nc+(ns-nc),1);
    else
        b = zeros(3*nb+nc+(ns-nc),1);
    end

    %% Formulate additional submatrices

    % Gat, E1, E2, and b
    scID = 0;
    idx = 1;
    for i=1:nc         
        C = sim.contacts(i);
        nsj = length(C.psi_n);
        body1id = sim.bodies(C.body1_id).bodyContactID;

        % Setup b (since we're already looping)
        b(3*nb+i) = C.psi_n(1) / sim.h;   % b is completed after A
        
        if nsj <= 1, scID = scID+1; continue; end

        Gn_i2 = zeros(3,nsj);
        for sc=1:nsj
            r2 = C.p1' + C.normal(sc,:)'*C.psi_n(sc) - sim.bodies(C.body2_id).pos;
            Gn_i2(:,sc) = [C.normal(:,sc); cross2d(r2, C.normal(:,sc))];
        end

        % Subcontact                            % TODO: This could be easily vectorized
        for sc=1:length(C.psi_n)
            scID = scID + 1; % Subcontact ID

            if sim.bodies(C.body1_id).dynamic
                if sc==1
                    Gn1T = Gn_i2(:,sc)';
                else
                    % Ga
                    GaT(scID-i,3*body1id-2:3*body1id) = Gn1T-Gn_i2(:,sc)';
                end
            end
        end


        if nsj > 1      % Both dynamic and static bodies contribute to E1 & E2.
            E1(i , idx:idx+nsj-2) = 1;                               % E1
            E2(idx:idx+nsj-2, idx:idx+nsj-2) = tril(ones( nsj-1 ));     % E2

            % CDA portion of b:
            bdex = 3*nb + nc + nd*nc + idx;
            if sc > 1
                b(bdex : bdex+nsj-2) = (C.psi_n(1)-C.psi_n(2:nsj)) / sim.h;
            end

            idx = idx + nsj - 1;
        end

    end

    %% Construct A and finish b
    if nd > 0 && sim.FRICTION 
        % Note: first row is negated
        A = [-M               +Gn                      +Gf         zeros(3*nb,(ns-nc)+nc)
             Gn'              zeros(nc,nc+nd*nc)       E1          zeros(nc,nc)
             Gf'              zeros(nd*nc,nc+nd*nc+(ns-nc))        E
             GaT              zeros(ns-nc,nc+nd*nc)    E2          zeros((ns-nc),nc)
            zeros(nc,3*nb)   U                        -E'         zeros(nc,(ns-nc)+nc) ];
    else % The case where friction is not included (No, not the most efficient).
        A = [-M       Gn                zeros(3*nb,(ns-nc))
             Gn'      zeros(nc,nc)      E1
             GaT      zeros(ns-nc,nc)   E2                    ];
    end

    activeBody = 1;
    for i=1:length(sim.bodies)
        if sim.bodies(i).active && sim.bodies(i).dynamic
           low = 3*activeBody-2; high = 3*activeBody;
           b(low:high) = M(low:high,low:high) * sim.bodies(i).nu + sim.h*sim.bodies(i).Fext;
           activeBody = activeBody + 1;
        end
    end
    
    
%     disp([' ST: rank(A) = ' num2str(rank(full(A))) ' on a ' num2str(size(A,1)) ' square matrix -> ' num2str(rank(full(A)) / size(A,1))]);
%   disp(['      ST: cond(A) = ' num2str(cond(full(A))) ' on a ' num2str(size(A,1)) ' square matrix ']);
%   disp(['      ST: trace(A) = ' num2str(trace(full(A))) ' on a ' num2str(size(A,1)) ' square matrix ']);

    
    
%     for i=1:nb          % b was started above, it's finished here.
%         j = sim.activeBodies(i);
%         low = 3*i-2; high =3*i;
%         b(low:high) = M(low:high,low:high) * sim.P{j}.nu + sim.h*sim.P{j}.Fext;
%     end

    
    % Solve the MCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);

    if sim.FRICTION && ~isempty(Gf)
      l = [-big*ones(3*nb,1);
           zeros((2+nd)*nc+(ns-nc),1)];
    else
      l = [-big*ones(3*nb,1)
       zeros(nc+(ns-nc),1)];
    end

    newNU = pathlcp(A,b,l,u,z0);

end




