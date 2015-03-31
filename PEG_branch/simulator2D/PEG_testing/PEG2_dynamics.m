

% Given a simulation object sim with a set of constraints, PEG_dynamics
% will formulate the time-stepping subproblem for PEG.  



function [newNU,sim] = PEG2_dynamics( sim )

    sim = get_all_contacts_2d( sim ); 
    if isempty(sim.contacts) && sim.num_jointConstraints <= 0
        newNU = []; return; 
    end
    sim = PEG_constraints_from_contacts( sim, -0.2 );  % eps_theta = 0.25
    
    %%%%%%%%%%%%%%%%%%%%%%%
    %% CONVERT Xconstraints to Iconstraints with copied contacts
    original_Icount = length(sim.Iconstraints); 
    for cID=1:length(sim.Xconstraints)
        
        if sim.contacts(sim.Xconstraints(cID).C1).applicability > -.2 && ...
           sim.contacts(sim.Xconstraints(cID).C2).applicability > -.2
       
            %disp([' Keeping Xconst on ' num2str(sim.Xconstraints(i).C1) ' and ' num2str(sim.Xconstraints(i).C2)]);
        
            % Copy the contacts
            sim.contacts(end+1) = sim.contacts(sim.Xconstraints(cID).C1);
            sim.contacts(end+1) = sim.contacts(sim.Xconstraints(cID).C2);

            % Create a new Icontstraint
            sim.Iconstraints(end+1) = constraint_inter_contact(length(sim.contacts)-1,length(sim.contacts));
            
        end
    end
    sim.Xconstraints = [];
    %%%%%%%%%%%%%%%%%%%%%%%
    

    %% Initialize matrices
    
    % Useful vars 
    C = sim.contacts; 
    nb = sim.num_activeBodies;    % Number of bodies with contacts

    % Init submatrices 
%     M =  sparse(3*nb,3*nb);  % Mass matrix
%     NU = sparse(3*nb,1);     % Velocity, including angular   
%     FX = sparse(3*nb,1);     % External force (not impulse!) 

    % M, NU, and FX
%     for b=1:length(sim.bodies)
%      B = sim.bodies(b);
%      if B.active && B.dynamic
%         cID = B.bodyContactID;
%         M(3*cID-2:3*cID,3*cID-2:3*cID) = diag([B.mass B.mass B.J]);  
%         NU(3*cID-2:3*cID) = B.nu;                  % NU
%         FX(3*cID-2:3*cID) = B.Fext;                % FX
%      end
%     end
    M = sim.dynamics.M;
    NU = sim.dynamics.NU;
    FX = sim.dynamics.FX; 
    
    
    U = sim.Uconstraints;
    I = sim.Iconstraints; 
    nu = length(U); 
    ni = length(I); 
    
    if sim.FRICTION
        nd = 2; 
        Un = sparse(nu,nu);
        Up = sparse(ni,ni);
        En = sparse(length(U),length(U));   
        Ep = sparse(length(I),length(I)); 
        Gfn = sparse(3*nb,2*length(U));    
        Gfp = sparse(3*nb,2*length(I));
    end
    
    
    %% Unilateral constraints
    Gn = sparse( 3*nb, length(sim.Uconstraints) ); 
    psi_n = zeros( length(sim.Uconstraints), 1 );
    for cID = 1:length(U)
        ci = C(U(cID).C);   % ith contact  
        
        B1 = sim.bodies(ci.body1_id);
        B2 = sim.bodies(ci.body2_id);
        r1 = ci.p1' - B1.pos;
        r2 = (ci.p1' + ci.psi_n*ci.normal') - B2.pos; 
        
        % Body 1 portion
        if B1.dynamic
            Gn(3*B1.bodyContactID-2:3*B1.bodyContactID, cID) = [ -ci.normal'
                                        cross2d( r1, -ci.normal' ) ];
        end
        
        % Body 2 portion
        if B2.dynamic
            Gn (3*B2.bodyContactID-2:3*B2.bodyContactID, cID) = [ ci.normal'
                                         cross2d( r2, ci.normal' ) ];
        end
        
        psi_n(cID) = ci.psi_n; 
        
        % Friction 
        if sim.FRICTION
            % E
            En(nd*cID-(nd-1):nd*cID,cID) = ones(nd,1);
            % U
            Un(cID,cID) = 0.5*B1.mu * B2.mu;
            % Gf
            % TODO: Select the initial tangent opposing motion
            d = [-ci.normal(2); ci.normal(1)];
            if B1.dynamic
                Gfn(3*B1.bodyContactID-2:3*B1.bodyContactID, nd*(cID-1)+1) = [d; cross2d(r1,d)];
                Gfn(3*B1.bodyContactID-2:3*B1.bodyContactID, nd*(cID-1)+2) = [-d; cross2d(r1,-d)];
            end
            if B2.dynamic
                Gfn(3*B2.bodyContactID-2:3*B2.bodyContactID, nd*(cID-1)+1) = [d; cross2d(r2,d)];
                Gfn(3*B2.bodyContactID-2:3*B2.bodyContactID, nd*(cID-1)+2) = [-d; cross2d(r2,-d)];
            end
        end
        
    end
    
    
    %% Inter-contact constraints
    % In 2D, we have the nice feature that all I-constraints have exactly 
    % 2 contacts.  We will utilize this throughout. 
    Gc = sparse( 3*nb, length(sim.Iconstraints) ); 
    Gp = sparse( 3*nb, 2*length(sim.Iconstraints) );   % This may be too large
    psi_c = zeros( length(sim.Iconstraints), 1 );
    psi_p = zeros( 2*length(sim.Iconstraints), 1 );  % This may be too large  
    
    di = 0;
    for cID = 1:length(I)
        
        c1 = C(I(cID).C1(1));     % With I-constraints, there is at least one iota contact
        if isempty(I(cID).C2)  
           c2 = C(I(cID).C1(2));
        else
           c2 = C(I(cID).C2);  
        end
        
        % IMPORTANT!  It is necessary that c2 is in the same frame as c1,
        % therefore, if c1 is from body A->B and c2 is from body B->A, we
        % must "flip" c2 to match c1.
        if c1.body1_id ~= c2.body1_id
            c_temp = c2;
            c2.body1_id = c_temp.body2_id;
            c2.body2_id = c_temp.body1_id;
            c2.p1 = c_temp.p1 + c_temp.psi_n * c_temp.normal;
            c2.normal = -c_temp.normal;
            c2.f1id = c_temp.f2id;
            c2.f2id = c_temp.f1id;
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
            Gc(3*B1.bodyContactID-2:3*B1.bodyContactID, cID) = G1a-G2a;
        end
        if B2.dynamic
            Gc (3*B2.bodyContactID-2:3*B2.bodyContactID, cID) = G1b-G2b;
        end
        psi_c(cID) = c1.psi_n - c2.psi_n; 
        
        
        % Gp
        if cID <= original_Icount
            % First contact
            di = di+1;
            if B1.dynamic
                Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G1a; 
            end
            if B2.dynamic
                Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G1b;  
            end
            psi_p(di) = c1.psi_n; 

            % Second contact
            if isempty(I(cID).C2)
                di = di+1;
                if B1.dynamic
                    Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G2a;
                end
                if B2.dynamic
                    Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G2b; 
                end
                psi_p(di) = c2.psi_n; 
            end
        else
            % Contact with smallest absolute gap distance
            %if c1.psi_n > c2.psi_n
            %if abs(c1.psi_n) < abs(c2.psi_n)
                di = di+1;
                if B1.dynamic
                    Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G1a; 
                end
                if B2.dynamic
                    Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G1b;  
                end
                psi_p(di) = c1.psi_n; 
%             else
%                 di = di+1;
%                 if B1.dynamic
%                     Gp(3*B1.bodyContactID-2:3*B1.bodyContactID, di) = G2a; 
%                 end
%                 if B2.dynamic
%                     Gp (3*B2.bodyContactID-2:3*B2.bodyContactID, di) = G2b;  
%                 end
%                 psi_p(di) = c2.psi_n; 
%             end
        end
        
        % Friction 
        if sim.FRICTION
            r1 = c1.p1' - B1.pos;           % TODO: could be done above
            r2 = (c1.p1' + c1.psi_n*c1.normal') - B2.pos;  
            
            % E
            Ep(nd*cID-(nd-1):nd*cID,cID) = ones(nd,1);
            % U
            Up(cID,cID) = 0.5*B1.mu * B2.mu;
            % Gf
            % TODO: Select the initial tangent opposing motion
            d = [-c1.normal(2); c1.normal(1)];
            if B1.dynamic
                Gfp(3*B1.bodyContactID-2:3*B1.bodyContactID, nd*(cID-1)+1) = [d; cross2d(r1,d)];
                Gfp(3*B1.bodyContactID-2:3*B1.bodyContactID, nd*(cID-1)+2) = [-d; cross2d(r1,-d)];
            end
            if B2.dynamic
                Gfp(3*B2.bodyContactID-2:3*B2.bodyContactID, nd*(cID-1)+1) = [d; cross2d(r2,d)];
                Gfp(3*B2.bodyContactID-2:3*B2.bodyContactID, nd*(cID-1)+2) = [-d; cross2d(r2,-d)];
            end
        end
        
    end
    
    % Here, we need to clip Gd and psi_d, and Gp and psi_p
    Gp = Gp(:,1:di); 
    psi_p = psi_p(1:di); 
    
   
    
    % Selection matrices
    Ec = speye(size(Gc,2)); 
    Epc = zeros(size(Gp,2),size(Gc,2));  % TODO: put in loop above 
    d = 1;
    for cID=1:length(I)  
        if isempty(I(cID).C2)
            Epc(d:d+1,cID) = [1;1]; 
            d = d+2;
        else
            Epc(d,cID) = 1; 
            d = d+1;
        end
    end
    
    nc = size(Gc,2);
    np = size(Gp,2);
    Gb = sim.dynamics.Gb; 
    njc = sim.num_jointConstraints; 
    
    %NUM_U = size(Gn,2);
%     NUM_I = size(Gp,2); 
%     %IoutOfAll = NUM_I/(NUM_I+NUM_U);
%     %disp([num2str(sim.step) '  num U: ' num2str(NUM_U) ', num I: ' num2str(NUM_I)  '   '  num2str(IoutOfAll)]);
%     %sim.userData.STcount(sim.step) = IoutOfAll;  
%     sim.userData.STcount(sim.step) = NUM_I;  
    

    h = sim.h;
    %% Construct PEG formulation
    if sim.FRICTION
        A = [ -M   Gb     Gn           Gp      zeros(3*nb, nc)    Gfn  Gfp  zeros(3*nb,nu+ni) 
              Gb'  zeros(njc,njc+nu+np+nc)                        zeros(njc, 3*(nu+ni))
              Gn'  zeros(nu,njc+nu+np+nc)                         zeros(nu, 3*(nu+ni))
              Gp'  zeros(np,njc+nu+np)         Epc                zeros(np, 3*(nu+ni))
              Gc'  zeros(nc,njc+nu+np)         Ec                 zeros(np, 3*(nu+ni)) 
              Gfn' zeros(2*nu,njc+nu+np+nc+2*(nu+ni))                       En  zeros(2*nu,ni)
              Gfp' zeros(2*ni,njc+nu+np+nc+2*(nu+ni)+nu)                        Ep
              zeros(nu, 3*nb+njc) Un   zeros(nu,ni+nc)           -En'  zeros(nu,3*ni+nu)
              zeros(ni,3*nb+njc+nu)     Up     zeros(ni,nc+2*nu)       -Ep' zeros(ni,nu+ni)    ];  
          
        b = [ M*NU + FX*h
              sim.dynamics.joint_bn
              psi_n/h
              psi_p/h 
              psi_c/h
              zeros(3*(nu+ni),1) ] ;  
    else
        A = [ -M   Gb     Gn           Gp      zeros(3*nb, nc)
              Gb'  zeros(njc,njc+nu+np+nc)
              Gn'  zeros(nu,njc+nu+np+nc)
              Gp'  zeros(np,njc+nu+np)         Epc                 
              Gc'  zeros(nc,njc+nu+np)         Ec       ];  
          
          
        b = [ M*NU + FX*h
              sim.dynamics.joint_bn
              psi_n/h
              psi_p/h 
              psi_c/h ] ;  
    end
      
      
    % Solve the PEG MLCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);
    l = zeros(size(u));
    l(1:3*nb+njc) = -big;

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
    
% disp(['  STEP ' num2str(sim.step)]);
%         sim.contacts(1).p1
%         sim.contacts(1).normal
%         sim.contacts(2).p1
%         sim.contacts(2).normal

end













