
function newNU = test_Binh( sim )

%sim.contacts = [sim.contacts(1) sim.contacts(2)]; 
%sim.Iconstraints = sim.Iconstraints(1); 

    %% Useful vars
    nb = sim.num_activeBodies;    % Number of bodies with contacts
    nc = size(sim.contacts,2);    % Number of contacts
    nxc = size(sim.Iconstraints,2);    % Number of cross-contact constraints 
    nd = sim.num_fricdirs;        % Number of directions in discrete friction "cone"
    
    ns = length(sim.contacts);
    for i=1:length(sim.contacts)
       ns = ns + length(sim.contacts(i).psi_n)-1; 
    end


    %% Init submatrices
    M = zeros(3*nb,3*nb);
    Gn = zeros(3*nb,nc);
    NU = zeros(3*nb,1);   % Velocity, including angular
    FX = zeros(3*nb,1);   % External force (not impulse!)
    PSI = zeros(nc,1);    % Gap distance per contact, psi_n
    sim.num_subContacts = 0;


    %% Calculate submatrices
    
    % M, NU, and FX
    for i=1:length(sim.bodies)
        B = sim.bodies(i);
        if B.active && B.dynamic
            cID = B.bodyContactID;
            M(3*cID-2:3*cID,3*cID-2:3*cID) = diag([B.mass B.mass B.J]);
            NU(3*cID-2:3*cID) = B.nu;                  % NU
            FX(3*cID-2:3*cID) = B.Fext;                % FX
        end
    end

    %% Formulate additional submatrices
    % mPEG matrices are combinations of contact wrenches.  Construct Gn for
    % body1 and body2 for each contact
    Gn1 = zeros(3,nc);
    Gn2 = zeros(3,nc);
    Gd = zeros(3*nb,nc);    
    Gc = zeros(3*nb, nxc);
    Ge = zeros(3*nb, nxc);
    Gp = zeros(3*nb, nc);
    psi_c = zeros(nxc,1); 
    psi_e = zeros(nxc,1);  
    psi_d = zeros(nc,1); 
    psi_p = zeros(nc,1); 
    Ed = eye(nc);
    Ec = eye(nxc);
    Ee = -eye(nxc);
    Eec = eye(nxc);
    Epd = eye(nc);
    Epc = zeros(nc,nxc);
    
    %% Iterate over contacts
    for i=1:nc
        C = sim.contacts(i); 
        n = C.normal;
        B1 = sim.bodies(C.body1_id);
        B2 = sim.bodies(C.body2_id); 
        r1 = C.p1 - B1.pos'; 
        r2 = C.p1 + C.psi_n*n - B2.pos'; 
        Gn1(:,i) = [-n'; cross2d(r1,-n)];
        Gn2(:,i) = [ n'; cross2d(r2, n)];
        
        % Begin to populate PSI
        PSI(i) = C.psi_n; 
        
        % While looping over the contacts, let's also form Gd
        Gd(3*C.body1_id-2:3*C.body1_id, i) = Gn1(:,i);
        Gd(3*C.body2_id-2:3*C.body2_id, i) = Gn2(:,i);
        psi_d(i) = PSI(i);
    end

    
    %% Iterate over inter-contact constraints
    for i=1:size(sim.Iconstraints,2)
        I = sim.Iconstraints(i); 
        b1id = sim.bodies(sim.contacts(I.c1).body1_id).bodyContactID;
        b2id = sim.bodies(sim.contacts(I.c2).body2_id).bodyContactID; 
        
        % Gc
        Gc(3*b1id-2:3*b1id, i) = Gn1(:,I.c2) - Gn1(:,I.c1);
        Gc(3*b2id-2:3*b2id, i) = Gn2(:,I.c2) - Gn2(:,I.c1);
        psi_c(i) =  PSI(I.c2) - PSI(I.c1); 
        
        % Ge
        Ge(3*b1id-2:3*b1id, i) = Gn1(:,I.c2);
        Ge(3*b2id-2:3*b2id, i) = Gn2(:,I.c2);
        psi_e(i) = PSI(I.c2);
        
        % Gp (ns entries per inter-contact constraint, assumed 2 for now)
        Gp(3*b1id-2:3*b1id, 2*i-1) = Gn1(:,I.c2);
        Gp(3*b2id-2:3*b2id, 2*i-1) = Gn2(:,I.c2);
        psi_p(2*i-1) = PSI(I.c2);
        
        Gp(3*b1id-2:3*b1id, 2*i) = Gn1(:,I.c2);
        Gp(3*b2id-2:3*b2id, 2*i) = Gn2(:,I.c2);
        psi_p(2*i) = PSI(I.c2);     % TODO TODO: isn't this meant to be psi_p(2*i-1:2*i) ?? 
        
        % Epc
        Epc(2*i-1:2*i,i) = 1;
    end

    %% Construct A and finish b
    % Currently, no friction constraints
    A = [ -M    zeros(3*nb,2*nxc+nc)           Gd
           Gc'  Ec   zeros(nxc,2*nc+nxc)
           Gd'  zeros(nc,nxc)  Ed    zeros(nc,nxc+nc)
           Ge'  Eec  zeros(nxc,nc)   Ee        zeros(nxc,nc)
           Gp'  Epc  Epd  zeros(nc,nxc+nc)   ];

    % Construct b
    activeBody = 1;
    for i=1:length(sim.bodies)
        if sim.bodies(i).active
           low = 3*activeBody-2; high = 3*activeBody;
           b(low:high) = M(low:high,low:high) * sim.bodies(i).nu + sim.h*sim.bodies(i).Fext;
           activeBody = activeBody + 1;
        end
    end
    
    h = sim.h; 
    b = [ b'
          psi_c / h
          psi_d / h
          psi_e / h
          psi_p / h ]; 
    
    % Solve the MCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);
    l = zeros(size(u));
    l(1:3*nb) = -big;

    
%     %% Solve by hand? 
%         % Assume that there will actually be no collision within time h
%     X = [ sim.bodies(1).nu
%           sim.bodies(2).nu 
%           -psi_c/h - Gc'*b(1:6)     % c
%           0     % d1
%           0     % d2
%           0     % e
%           0     % pn1
%           0 ];  % pn2
    
    %% Attempt to solve with PATH
    newNU = pathlcp(A,b,l,u,z0);



end













