

% Given a simulation object sim with a set of constraints, PEG_dynamics
% will formulate the time-stepping subproblem for PEG.  


function newNU = PEG_dynamics( sim )

    sim = get_all_contacts_2d( sim ); 
    if isempty(sim.contacts)
        newNU = []; return; 
    end
    sim = PEG_constraints_from_contacts( sim, -0.25 );  % eps_theta = 0.25
    
            %% TODO TODO TODO: REMOVE REMOVE 
            %sim.Xconstraints = [];  
            if min([sim.contacts.psi_n]) < 0
                disp(min([sim.contacts.psi_n]))
            end


    %% Initialize matrices
    
    % Useful vars 
    C = sim.contacts; 
    nb = sim.num_activeBodies;    % Number of bodies with contacts

    % Init submatrices 
    M =  sparse(3*nb,3*nb);  % Mass matrix
    NU = sparse(3*nb,1);     % Velocity, including angular   
    FX = sparse(3*nb,1);     % External force (not impulse!) 

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
    
    
    %% Unilateral constraints
    Gn = sparse( 3*nb, length(sim.Uconstraints) ); 
    psi_n = zeros( length(sim.Uconstraints), 1 );
    
    U = sim.Uconstraints;
    for i = 1:length(U)
        ci = C(U(i).C);   % ith contact  
        
        B1 = sim.bodies(ci.body1_id);
        B2 = sim.bodies(ci.body2_id);
        
        % Body 1 portion
        if B1.dynamic
            Gn(3*B1.bodyContactID-2:3*B1.bodyContactID, i) = [ -ci.normal'
                                        cross2d( ci.p1' - B1.pos, -ci.normal' ) ];
        end
        
        % Body 2 portion
        if B2.dynamic
            Gn (3*B2.bodyContactID-2:3*B2.bodyContactID, i) = [ ci.normal'
                                         cross2d( (ci.p1' + ci.psi_n*ci.normal') - B2.pos, ci.normal' ) ];
        end
        
        psi_n(i) = ci.psi_n; 
        
    end
    
    
    %% Inter-contact constraints
    % In 2D, we have the nice feature that all I-constraints have exactly 
    % 2 contacts.  We will utilize this throughout. 
    Gc = sparse( 3*nb, length(sim.Iconstraints) ); 
    Gd = sparse( 3*nb, 2*length(sim.Iconstraints) );   % This may be too large
    Ge = sparse( 3*nb, length(sim.Iconstraints) );
    Gp = sparse( 3*nb, 2*length(sim.Iconstraints) );   % This may be too large
    psi_c = zeros( length(sim.Iconstraints), 1 );
    psi_d = zeros( 2*length(sim.Iconstraints), 1 );  % This may be too large 
    psi_e = zeros( length(sim.Iconstraints), 1 ); 
    psi_p = zeros( 2*length(sim.Iconstraints), 1 );  % This may be too large  
    
    I = sim.Iconstraints; 
    di = 0;
    for i = 1:length(I)
        
        c1 = C(I(i).C1(1));     % With I-constraints, there is at least one iota contact
        if isempty(I(i).C2)  
           c2 = C(I(i).C1(2));
        else
           c2 = C(I(i).C2);  
        end
        
        B1 = sim.bodies(c1.body1_id);
        B2 = sim.bodies(c1.body2_id);
        
        G1a = [ -c1.normal'
                cross2d( c1.p1' - B1.pos, -c1.normal' ) ];
        G1b = [ c1.normal'
                cross2d( (c1.p1' + c1.psi_n*c1.normal') - B2.pos, c1.normal' ) ]; 
        G2a = [ -c2.normal'
                cross2d( c2.p1' - B1.pos, -c2.normal' ) ];
        G2b = [ c2.normal'
                cross2d( (c2.p1' + c2.psi_n*c2.normal') - B2.pos, c2.normal' ) ]; 
        
        % Gc
        if B1.dynamic
            Gc(3*B1.bodyContactID-2:3*B1.bodyContactID, i) = G1a-G2a;
        end
        if B2.dynamic
            Gc (3*B2.bodyContactID-2:3*B2.bodyContactID, i) = G1b-G2b;
        end
        psi_c(i) = c1.psi_n - c2.psi_n; 
        
        % Ge
        if B1.dynamic
            Ge(3*B1.bodyContactID-2:3*B1.bodyContactID, i) = G1a; 
        end
        if B2.dynamic
            Ge (3*B2.bodyContactID-2:3*B2.bodyContactID, i) = G1b; 
        end
        psi_e(i) = c1.psi_n; 
        
        % Gd and Gp
        % First contact
        di = di+1;
        if B1.dynamic
            Gd(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G1a; 
            Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G1a; 
        end
        if B2.dynamic
            Gd (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G1b;
            Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G1b;  
        end
        psi_d(di) = c1.psi_n; 
        psi_p(di) = c1.psi_n; 
        
        % Second contact
        if isempty(I(i).C2)
            di = di+1;
            if B1.dynamic
                Gd(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G2a;
                Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G2a;
            end
            if B2.dynamic
                Gd (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G2b; 
                Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G2b; 
            end
            psi_d(di) = c2.psi_n; 
            psi_p(di) = c2.psi_n; 
        end
    end
    
    % Here, we need to clip Gd and psi_d, and Gp and psi_p
    Gd = Gd(:,1:di);
    Gp = Gp(:,1:di); 
    psi_d = psi_d(1:di);
    psi_p = psi_p(1:di); 
    
    
    
    
    %% Cross-contact constraints 
    % In 2D, we have the nice propertie for X-constraints that |C1| = |C2| = 1  
    X = sim.Xconstraints; 
    ni = length(I);                 % The offset in Gc and Ge from the I-constraints
    Gc = [Gc zeros(3*nb,length(X))];  % Add room for X-constraints
    Ge = [Ge zeros(3*nb,length(X))]; 
    psi_c = [psi_c; zeros(length(X),1)];
    psi_e = [psi_e; zeros(length(X),1)];
    
    for i = 1:length(X)
        
        c1 = C(X(i).C1);
        c2 = C(X(i).C2); 
        
        B1 = sim.bodies(c1.body1_id);
        B2 = sim.bodies(c1.body2_id);
        
        G1a = [ -c1.normal'
                cross2d( c1.p1' - B1.pos, -c1.normal' ) ];
        G1b = [ c1.normal'
                cross2d( (c1.p1' + c1.psi_n*c1.normal') - B2.pos, c1.normal' ) ]; 
        G2a = [ -c2.normal'
                cross2d( c2.p1' - B1.pos, -c2.normal' ) ];
        G2b = [ c2.normal'
                cross2d( (c2.p1' + c2.psi_n*c2.normal') - B2.pos, c2.normal' ) ]; 
        
        % Gc
        if B1.dynamic
            Gc(3*B1.bodyContactID-2:3*B1.bodyContactID, ni+i) = G1a-G2a;
        end
        if B2.dynamic
            Gc (3*B2.bodyContactID-2:3*B2.bodyContactID, ni+i) = G1b-G2b;
        end
        psi_c(ni+i) = c1.psi_n - c2.psi_n; 
        
        % Ge
        if B1.dynamic
            Ge(3*B1.bodyContactID-2:3*B1.bodyContactID, ni+i) = G1a; 
        end
        if B2.dynamic
            Ge (3*B2.bodyContactID-2:3*B2.bodyContactID, ni+i) = G1b; 
        end
        psi_e(ni+i) = c1.psi_n;      
    end
    
    
    % Selection matrices
    Ec = speye(size(Gc,2)); 
    Ed = speye(size(Gd,2)); 
    Ee = speye(size(Ge,2));
    Eec = Ee;   
    Epd = speye(size(Gp,2)); 
    Epc = zeros(size(Gp,2),size(Gc,2));  % TODO: put in loop above 
    d = 1;
    for i=1:length(I)  
        if isempty(I(i).C2)
            Epc(d:d+1,i) = [1;1]; 
            d = d+2;
        else
            Epc(d,i) = 1; 
            d = d+1;
        end
    end
    
    nu = length(U); 
    nc = size(Gc,2);
    nd = size(Gd,2); 

    %% Construct PEG formulation
    A = [ -M   Gn            zeros(3*nb, 2*nc+nd)             Gp
          Gn'  zeros(nu,nu+2*nc+2*nd)
          Gc'  zeros(nc,nu)  Ec       zeros(nc,2*nd+nc)
          Gd'  zeros(nd,nu+nc)        Ed            zeros(nd,nc+nd)
          Ge'  zeros(nc,nu)  Eec      zeros(nc,nd)  -(10^-5)*Ee        zeros(nc,nd)
          Gp'  zeros(nd,nu)  Epc      Epd           zeros(nd,nc+nd)       ];  
          
      
    h = sim.h;
    b = [ M*NU + FX*h
          psi_n/h
          psi_c/h
          psi_d/h
          psi_e/h
          psi_p/h  ] ;  
      
      
%     disp([' ST: rank(A) = ' num2str(rank(full(A))) ' on a ' num2str(size(A,1)) ' square matrix -> ' num2str(rank(full(A)) / size(A,1))]);
%   disp(['      ST: cond(A) = ' num2str(cond(full(A))) ' on a ' num2str(size(A,1)) ' square matrix ']);
%   disp(['      ST: trace(A) = ' num2str(trace(full(A))) ' on a ' num2str(size(A,1)) ' square matrix ']);

      
      
    % Solve the PEG MLCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);
    l = zeros(size(u));
    l(1:3*nb) = -big;

    try 
        newNU = pathlcp(A,b,l,u,z0);
    catch
        % Initialize z0 with current body velocities 
        for bid=1:length(sim.bodies)
           B = sim.bodies(bid);
           if B.dynamic && B.active, z0(3*B.bodyContactID-2:3*B.bodyContactID) = B.nu; end
        end 
        
        newNU = pathlcp(A,b,l,u,z0);
        disp('Path failed with z0 of zeros.  Retrying with initialized z0...');
    end
    
    
%         z = A*newNU+b;
%         p_n = newNU(1+size(M,1): size(M,1)+size(Gn,2))
%         p_p = newNU(1+size(M,1)+size(Gc,2)+size(Gd,2)+size(Ge,2): size(M,1)+size(Gc,2)+size(Gd,2)+size(Ge,2)+size(Gp,2))
    
disp(['  STEP ' num2str(sim.step)]);
        sim.contacts(1).p1
        sim.contacts(1).normal
        sim.contacts(2).p1
        sim.contacts(2).normal

end













