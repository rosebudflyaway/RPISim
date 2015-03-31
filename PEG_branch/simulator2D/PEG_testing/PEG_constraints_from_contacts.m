

% There are two cases of contact in 2D: vertex-edge or vertex-vertex.  

function sim = PEG_constraints_from_contacts( sim, eps_theta )

    C = sim.contacts; 
    U = []; % Unilateral constraints
    I = []; % Inter-contact constraints
    X = []; % Cross-contact constraints 
    
    COUNT_ST_TRAP = 0;
    COUNT_NO_TRAP = 0; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Given a set of contacts C, and a starting index i_start, returns the
    % subset of adjacent contacts sharing the same f1id.  
    function sub_c = getContactSet(C,i_start)
        f1id = C(i_start).f1id;  
        b1id = C(i_start).body1_id;  
        b2id = C(i_start).body2_id;
        n_c = length(C);
        i_end = i_start;  
        while i_end < n_c 
           if C(i_end+1).body1_id == b1id && C(i_end+1).body2_id == b2id && C(i_end+1).f1id == f1id
                i_end = i_end + 1;  
           else
               break;
           end
        end
        sub_c = C(i_start:i_end);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C2 = [];
    n_c = length(C);
    ci = 1;
    while ci <= n_c
       c = getContactSet(C,ci); 
       c_increment = length(c); 
       
       if length(c) == 1 
          if c.applicability > eps_theta
              U = [U constraint_unilateral(ci)];  % Unilateral 
              C2 = [C2 c];
              COUNT_NO_TRAP = COUNT_NO_TRAP + 1; 
          end
       else
          % Put contact set in order, and trim it
          c = PEG_trim_contact_set(c,eps_theta);  
          
          if length(c) > 2
             disp(['Length(c) = ' num2str(length(c))]);
             2; 
          end
          
          % Recheck for single contact or null contact
          if isempty(c)
             ci = ci + c_increment;
             continue;
          elseif length(c) == 1 
             C2 = [C2 c];
             U = [U constraint_unilateral(ci)];  % Unilateral 
          else
             C2 = [C2 c]; 
             
             % Identify the four contacts involved with each interior
             % vertex-vertex pair, and creat an inter and cross contact
             % constraint for each pair.  
             
             for vb_index = 1 : length(c)-1
                 
               % TODO: this search would not be necessary if contacts kept ID's (as silly as that seems)
               c1 = get_contactID(sim,1,c(1).body1_id,c(1).body2_id,c(1).f1id,c(vb_index  ).f2id);
               c2 = get_contactID(sim,1,c(1).body1_id,c(1).body2_id,c(1).f1id,c(vb_index+1).f2id);
               
               % Test if swap is needed due to vertex indexing wrap of B
               if C(c1).f2id == 1 && C(c2).f2id > 2
                   c_temp = c1;
                   c1 = c2;
                   c2 = c_temp;
               end
               
               % c3
               if c(1).f1id == 1
                   c3_f1id = sim.bodies(c(1).body1_id).num_verts;
               else
                   c3_f1id = c(1).f1id - 1;
               end
               c3 = get_contactID(  sim, ...            % sim
                                    1, ...              % start at 
                                    c(1).body2_id, ...  % Body 2 id
                                    c(1).body1_id, ...  % Body 1 id
                                    C(c2).f2id, ...     % Body 2 feature id
                                    c3_f1id );          % Body 1 feature id

               
               % c4
               c4 = get_contactID(  sim, ...
                                    1, ...
                                    c(1).body2_id, ...
                                    c(1).body1_id, ...
                                    C(c2).f2id, ...
                                    c(1).f1id); 
               
               
               %% Catch the case where one of the c3 or c4 contacts does not exist
               % This can occur because collision detection tests
               % vertex-segment distances within epsilon, and it may be
               % possible for two vertices of one edge to be within epsilon
               % of a second edge, without it being vice versa. 
               if ~c3 || ~c4
                   % In such a case, we simplify from a vertex-vertex case into one or two unilateral cases.  
                   if C(c1).applicability > C(c2).applicability
                      U = [U constraint_unilateral(c1)]; 
                   else
                      U = [U constraint_unilateral(c2)];
                   end
                   
                   % It is possible that niether c3 nor c4 is a valid contact
                   if c3
                      U = [U constraint_unilateral(c3)];
                   elseif c4
                      U = [U constraint_unilateral(c4)];
                   end
                   
                   continue; 
               end
               
               
               %% Check for the case where we could be replacing an I and X constraint with a single U(Ci)
               % Determine if one of the vertices meets the following criteria:
               %   psi > -10^-4 
               %   In Voronoi region 
%                if C(c1).psi_n >= -10^(-3) && inVoronoiEpsilonRegion(C(c1).p1, sim.bodies(C(c1).body2_id), C(c1).f2id)
%                    U = [U constraint_unilateral(c1)];
%                elseif C(c4).psi_n >= -10^(-3) && inVoronoiEpsilonRegion(C(c4).p1, sim.bodies(C(c4).body2_id), C(c4).f2id)
%                    U = [U constraint_unilateral(c4)];
%                elseif C(c2).psi_n >= -10^(-3) && inVoronoiEpsilonRegion(C(c2).p1, sim.bodies(C(c2).body2_id), C(c2).f2id)
%                    U = [U constraint_unilateral(c2)];
%                elseif C(c3).psi_n >= -10^(-3) && inVoronoiEpsilonRegion(C(c3).p1, sim.bodies(C(c3).body2_id), C(c3).f2id)
%                    U = [U constraint_unilateral(c3)];
%                end
               
%                if C(c1).psi_n >= -10^(-4) && inVoronoiEpsilonRegion(C(c1).p1, sim.bodies(C(c1).body2_id), C(c1).f2id) || ...
%                    C(c4).psi_n >= -10^(-4) && inVoronoiEpsilonRegion(C(c4).p1, sim.bodies(C(c4).body2_id), C(c4).f2id)
%                        
%                    U = [U constraint_unilateral(c1)]; 
%                    %U = [U constraint_unilateral(c4)];
%                    % TODO: This may still require a X-constraint
%                elseif C(c2).psi_n >= -10^(-4) && inVoronoiEpsilonRegion(C(c2).p1, sim.bodies(C(c2).body2_id), C(c2).f2id) || ...
%                        C(c3).psi_n >= -10^(-4) && inVoronoiEpsilonRegion(C(c3).p1, sim.bodies(C(c3).body2_id), C(c3).f2id)
%                        U = [U constraint_unilateral(c2)]; 
%                        U = [U constraint_unilateral(c3)]; 
%                end
               
               % If one does, pick the one that "most" does
               
               
               % Create a single U-constraint, drop all the others
               
               
               
               %% Inter contact constraints
               S1_temp = [];
               S2_temp = [];
               if C(c1).applicability > eps_theta
                   S1_temp = c1;
               else
                   S2_temp = c1;
               end
               if C(c2).applicability > eps_theta
                   S1_temp = [S1_temp c2];
               else
                   S2_temp = [S2_temp c2];
               end
               
               S3_temp = [];
               S4_temp = [];
               if C(c3).applicability > eps_theta
                   S3_temp = c3;
               else
                   S4_temp = c3;
               end
               if C(c4).applicability > eps_theta
                   S3_temp = [S3_temp c4];
               else
                   S4_temp = [S4_temp c4];
               end
               
               
               % Check to avoid adding redundant I and X constraints
               redundant = false;
               if C(c1).body1_id > C(c1).body2_id
                   for i=1:length(I)
                      if isequal(I(i).C1,S1_temp) && isequal(I(i).C2,S2_temp) || ...
                         isequal(I(i).C1,S3_temp) && isequal(I(i).C2,S4_temp) 
                          redundant = true;
                          break;
                      end
                   end
               end
               
               if ~redundant
                   I = [ I ...
                         constraint_inter_contact(S1_temp,S2_temp) ... 
                         constraint_inter_contact(S3_temp,S4_temp)     ]; 



                   %% Cross contact constraints
                   X = [ X ...
                         constraint_cross_contact(c1,c4) ...
                         constraint_cross_contact(c2,c3)     ]; 
                     
                   COUNT_ST_TRAP = COUNT_ST_TRAP + 1; 
               end
               
             end
          end
       end
       ci = ci + c_increment;
    end
    
    % DON'T Drop removed contacts
    %sim.contacts = C2; 
    
    
    
    %% Remove unilateral constraints that have contacts that are also included 
    % in inter-contact constraints.  
    for i=1:length(I)
        Icontacts = [I(i).C1 I(i).C2];
        for u=1:length(U)
           if U(u).C == Icontacts(1) || U(u).C == Icontacts(2)
               U(u) = [];  
               disp(['Dropping U ' num2str(u) ]);
               break; 
           end
        end
    end
    
    
    %% Check for and fix redundant unilateral constraints
    toRemove = [];
    for i=1:length(U)
        for j=i+1:length(U)
            if U(i).C == U(j).C
                toRemove = [toRemove j];
            end
        end
    end
    U(toRemove) = [];
    
    
    %% Remove inter-contact and cross-contact constraints where C1 is empty 
    dex = [];
    for i=1:length(I)
       if isempty(I(i).C1)
          dex = [dex i]; 
       end
    end
    I(dex) = [];
    X(dex) = []; 
    
    
    %% Apply some heuristics to the I-constraints
    for i=1:length(I)
        try
           if sim.contacts(I(i).C1).psi_n < -10^-3 && ...
              sim.contacts(I(i).C2).psi_n > sim.contacts(I(i).C1).psi_n && ...
              sim.contacts(I(i).C2).psi_n > -10^-3
                % Swap C1 and C2
                Ctemp = I(i).C1; 
                I(i).C1 = I(i).C2;
                I(i).C2 = Ctemp; 
           end
        catch
            % There is more than one contact in I(i).C1
            if sim.contacts(I(i).C1(1)).psi_n < -10^-3 && ...
              sim.contacts(I(i).C1(2)).psi_n > sim.contacts(I(i).C1(1)).psi_n && ...
              sim.contacts(I(i).C1(2)).psi_n > -10^-3
                % Swap C1 and C2
                Ctemp = I(i).C1(1); 
                I(i).C1(1) = I(i).C1(2);
                I(i).C1(2) = Ctemp; 
           end
        end
    end
    
    %% Apply some heuristics to the X-constraints
    for i=1:length(X)
       if sim.contacts(X(i).C1).psi_n < -10^-3 && ...
          sim.contacts(X(i).C2).psi_n > sim.contacts(X(i).C1).psi_n && ...
          sim.contacts(X(i).C2).psi_n > -10^-3
            % Swap C1 and C2
            Ctemp = X(i).C1; 
            X(i).C1 = X(i).C2;
            X(i).C2 = Ctemp; 
       end
    end
    
    
    %% With heuristic method, we only allow one possible force i.e. |C_i| = 1
    for i=1:length(I)
       if length(I(i).C1) > 1
          I(i).C2 = I(i).C1(2);
          I(i).C1(2) = []; 
       end
    end
    
    
    sim.Uconstraints = U;
    sim.Iconstraints = I;
    sim.Xconstraints = X; 
    
    sim.userData.ST_trap_count(sim.step) = COUNT_ST_TRAP;
    sim.userData.ST_trap_count(sim.step) = COUNT_NO_TRAP; 

end















