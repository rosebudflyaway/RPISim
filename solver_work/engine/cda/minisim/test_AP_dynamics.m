%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_AP_dynamics.m
%
% Anitescu-Potra dynamics formulation in 2D

function [NUnew, log_data] = test_ST_edges(B, V)
% Even though we take in V as a (x,y) point location, let's treat it as the 
% bottom vertex in a square where the center of mass is 1 unit directly
% above V.  That is: r = [0; -1].  This will give us a better sense of the
% 2-D as opposed to the point problem (maybe make the MCP dimensions more
% clear).  



i_debug = 0; % debugging level (controls amount of messages printed)



% Calculate minimum epsilon
%  V.min_eps = test_ST_calc_mineps(B);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set up the number of bodies, manifolds, contacts, and subcontacts,
%% within distance epsilon from the active vertex

cur_eps = test_CDA_calc_epsilon(V);

psi_eps = cur_eps / 10; % maximum penetration depth
% (penetration is ignored past this depth)



num_active_bodies = 1; % (the particle)


% apply broadphase collision detection
[num_manifolds, num_subcontacts, B_eps] = test_CDA_broadphase(B, V, cur_eps);
num_contacts = sum(num_subcontacts); % total number of contacts


if (i_debug > 0)
  fprintf(1, 'Number of manifolds: %d\n', num_manifolds);
  fprintf(1, 'Number of contacts: %d\n', num_contacts);
  fprintf(1, 'Number of subcontacts:\n');
  for incr_sc = 1:num_manifolds
    fprintf(1, '   [%d] %d\n', incr_sc, num_subcontacts(incr_sc));
  end
end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% It's dynamics time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






% Construct matrices A and b for the MCP



fricdirs = 2;   % friction directions (2D)

% Init submatrices 
% MM = zeros(3*num_bodies);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Initialize matrices


% Normal contact wrenches
Gn = zeros(2*num_active_bodies, num_contacts);



% Friction wrenches
Gf = zeros(2*num_active_bodies, fricdirs * num_contacts);

% Friction pick matrices
U = eye(num_contacts);
E = zeros(fricdirs*num_contacts, num_contacts); 


% b matrix
if (V.DEFINE_FRICTION)
    b = zeros(2 * num_active_bodies + num_contacts * (fricdirs + 2), 1);
else
    b = zeros(2 * num_active_bodies + num_contacts, 1);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







% Build Inertia submatrix, MM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%MM(1:3,1:3) = V.m * eye(3);
%  MM = diag([V.m V.m 1]);  % Dumby moment of inertia of 1
MM = V.m * eye(2); % with no moment of inertia/rotation


% Calculate viscous forces
V.Fv = -V.b * V.nu;

b(1:2,1) = MM * V.nu + V.h * V.Fext + V.h * V.Fv; %  no moment of inertia/rotation




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build subcontact dependent submatrices (Gn, Gf, Ga, U, E, E1, E2, b):


incr_Gn = 1; % incrementer for normal contact wrench

for incr_m = 1:num_manifolds

    edges_vertices = zeros(num_subcontacts(incr_m), 4);
    psi = zeros(num_subcontacts(incr_m), 1);


    for incr_s = 1:num_subcontacts(incr_m)

        % put together vertex data for the edges in this manifold
        edges_vertices(incr_s,:) = [B_eps(incr_m).x(incr_s), ...
                                    B_eps(incr_m).y(incr_s), ...
                                    B_eps(incr_m).x(incr_s+1), ...
                                    B_eps(incr_m).y(incr_s+1)];



        % Calculate gap distances
        psi(incr_s) = test_CDA_calc_gap_distance(V, edges_vertices(incr_s,:));
    

    
%          if (psi(incr_s) < -psi_eps)
        if (0) % DISABLE
            % penetration is too deep, ignore this constraint to prevent a blow up

            % shink the number of contacts and subcontacts
            num_subcontacts(incr_m) = num_subcontacts(incr_m) - 1;
            num_contacts = num_contacts - 1;
    
            % shrink Gn, Gf, E, U, and b
            Gn(:,end) = [];
            Gf = Gf(:,1:end-fricdirs);
        

    
            U = U(1:end-1, 1:end-1);
            E = E(1:end - fricdirs, 1:end -1);

            b = b(1:end - fricdirs - 2);
            
%              Gf = Gf(:, 1:size(Gf,2) - fricdirs);
%              U = U(1:size(U,1) - 1, 1:size(U,2) - 1);
%              E = E(1:size(E,1) - fricdirs, 1:size(E,2) - 1);
        
        else


            % build Gn, Gf, E, and b in here
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    
            % calculate tangent, normal, and intersect for this edge
            this_tangent = [edges_vertices(incr_s,3) - edges_vertices(incr_s,1); ...
                            edges_vertices(incr_s,4) - edges_vertices(incr_s,2)];
            this_tangent = this_tangent / norm(this_tangent);
            
  
            this_normal = [this_tangent(2); -this_tangent(1)];
            this_normal = this_normal / norm(this_normal);
  
  
            %this_r = [V.x; V.y; 0] + psi(incr_s) * -this_normal; % assuming body frame is the same as world frame
            % r is meant to be the vector from the center of mass of the body to the
      % point of contact, in the object's reference frame. 
            this_r = [0; -1];  % See top of file for why I chose [0; -1];  

    
    
            if (V.DEFINE_FRICTION)
                % Build Friction Submatrices (Gf, E, U)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

  
    %              rxd = this_r(1) * this_tangent(2) - this_r(2) * this_tangent(1);
    %              rxd_neg = this_r(1) * -this_tangent(2) - this_r(2) * -this_tangent(1);
            
    %              Gf_i1 = [[this_tangent; rxd] / norm([this_tangent; rxd]), ...
    %                  [-this_tangent; rxd_neg] / norm([-this_tangent; rxd_neg])];
            
                Gf_i1 = [[this_tangent] / norm([this_tangent]), ...
                        [-this_tangent] / norm([-this_tangent])];

                Gf(:,fricdirs * incr_s - (fricdirs - 1):fricdirs * incr_s) = Gf_i1;
        
    
    
                % Build E
                E(fricdirs*incr_s-(fricdirs-1):fricdirs*incr_s,incr_s) = ones(fricdirs,1);
    
                % Build U (assume uniform coefficient of friction)
                U(incr_s,incr_s) = V.mu;
    
            end

    
            % Build Gn
            %%%%%%%%%%
            %      rxn = this_r(1) * this_normal(2) - this_r(2) * this_normal(1);
            %    Gn_i = [-this_normal; rxn] / norm([-this_normal; rxn]);
            Gn_i = -this_normal; % no moment of inertia/rotation
    
  
            % (A-P constraints)
            Gn(:,incr_Gn) = Gn_i;
%              b(2 * num_active_bodies + incr_Gn, 1) = psi(incr_s) / V.h;
           %FIXME: check to see if b(..) = 0 is the only change required for the A-P method
            
  
            
            incr_Gn = incr_Gn + 1;
      
        end % if (psi(incr_s) < -psi_eps)
    
    end % for incr_s = 1:num_subcontacts
end % for incr_m = 1:num_manifolds





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% smallA = [-MM, Gn, Gf, zeros(size(MM,1), size(E1,2) + size(E,2))];

if (i_debug > 2)
    disp('MM = ');
    disp(MM);

    disp('Gn = ');
    disp(Gn);

    disp('Gf = ');
    disp(Gf);

end


big = 1e20;

if (V.DEFINE_FRICTION)
    A = [-MM, Gn, Gf, zeros(size(MM,1), size(E,2)); ...
         Gn', zeros(size(Gn,2), size(Gn,2) + size(Gf,2) + size(E,2)); ...
         Gf', zeros(size(Gf,2), size(Gn,2) + size(Gf,2)), E; ...
         zeros(size(U,1), size(MM,2)), U, -E', zeros(size(U,1), size(E,2))];
else
    A = [-MM, Gn; ...
         Gn', zeros(size(Gn,2))];
end


z0 = zeros(size(A,2),1); % initial guess
u = big * ones(size(A,2),1); % upper limit
l = zeros(size(A,2),1); % lower limit
l(1:2) = -big * ones(2,1);



if (i_debug > 3)
    fprintf(1, 'A [%d x %d] =\n', size(A,1), size(A,2));
    disp(A);

    fprintf(1, 'b [%d] =\n', size(b,1));
    disp(b);
end


% fprintf(1, 'Size of A: %d x %d\n', size(A,1), size(A,2));
% fprintf(1, 'Size of b: %d x %d\n', size(b,1), size(b,2));
% fprintf(1, 'Size of z0: %d x %d\n', size(z0,1), size(z0,2));
% fprintf(1, 'Size of u: %d x %d\n', size(u,1), size(u,2));
% fprintf(1, 'Size of l: %d x %d\n', size(l,1), size(l,2));



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve the MCP
%  t_start = cputime;
t_start = tic;
[z, ~] = pathlcp(A, b, l, u, z0);
%  t_solver = cputime - t_start;
t_solver = toc(t_start);


%[z, ~] = lemke(A, b, z0);  % You can't use Lemke to solve an MCP...
%NUnew = V.nu + MinvGn * z; % Part of LCP solution (not needed). 

NUnew = z(1:2);

if (i_debug > 1)
    disp('z =');
    disp(z);

    disp('nu =');
    disp(NUnew);
end


log_data.t_solver = t_solver;
log_data.MCPsize = size(A,1);

