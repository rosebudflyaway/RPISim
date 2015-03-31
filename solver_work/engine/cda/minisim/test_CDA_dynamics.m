%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_dynamics.m
%
% CDA dynamics formulation in 2D

function [NUnew, log_data] = test_CDA_dynamics(B, V)
% Even though we take in V as a (x,y) point location, let's treat it as the 
% bottom vertex in a square where the center of mass is 1 unit directly
% above V.  That is: r = [0; -1].  This will give us a better sense of the
% 2-D as opposed to the point problem (maybe make the MCP dimensions more
% clear).  



i_debug = 0; % debugging level (controls amount of messages printed)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set up the number of bodies, manifolds, contacts, and subcontacts

cur_eps = test_CDA_calc_epsilon(V);


num_active_bodies = 1;
num_bodies = length(B); % number of bodies
num_manifolds = num_bodies; % number of manifolds (one manifold per body here)

for incr_b = 1:num_bodies
  if (length(B(incr_b).x) ~= length(B(incr_b).y))
    error('Dimension mismatch for body vertex vectors');
    % go sit in the corner, and think about your life
  end
end
  


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
Gn = zeros(2*num_active_bodies, num_manifolds);

% Auxiliary contact wrenches
GaT = zeros(num_contacts - num_manifolds, 2*num_active_bodies);


% Friction wrenches
Gf = zeros(2*num_active_bodies, fricdirs * num_contacts);

% Friction pick matrices
U = eye(num_contacts, num_manifolds);
E = zeros(fricdirs*num_contacts, num_contacts);  % FIXME: num_contacts --> num_manifolds?

% Pick matrices E1 and E2:

% Rules for E1 and E2:
% E1 - ones
% E2 - lower triangular ones
% E1 and E2 have the same number of columns
% E2 is square
% E1 has the same number of rows as GnT
% E2 has the same number of rows as GaT

E1 = zeros(num_manifolds, num_contacts - num_manifolds);
E2 = zeros(num_contacts - num_manifolds, num_contacts - num_manifolds);


% padding matrices (zeros)
z1 = zeros(2*num_active_bodies, num_contacts - num_manifolds);
z2 = zeros(num_manifolds, num_manifolds);
z3 = zeros(num_contacts - num_manifolds, num_manifolds);


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


incr_Ecol = 1; % incrementer for column of E1, row,col of E2
incr_GaT = 1; % incrementer for auxiliary contact wrench
incr_Gn = 1; % incrementer for normal contact wrench

for incr_m = 1:num_manifolds
    
  edges_vertices = zeros(num_subcontacts(incr_m), 4);
  psi = zeros(num_subcontacts(incr_m), 1);

  % crawl through psi, and find the subcontact with the minimum positive psi to choose as the active edge
  % not using Matlab's min() function.
  min_psi = inf; % infinity
  min_psi_idx = 1;

  for incr_s = 1:num_subcontacts(incr_m)

    % put together vertex data for the edges in this manifold
    edges_vertices(incr_s,:) = [B_eps(incr_m).x(incr_s), ...
                                B_eps(incr_m).y(incr_s), ...
                                B_eps(incr_m).x(incr_s+1), ...
                                B_eps(incr_m).y(incr_s+1)];

    % Calculate gap distances
    psi(incr_s) = test_CDA_calc_gap_distance(V, edges_vertices(incr_s,:));

    if (psi(incr_s) < min_psi) && (psi(incr_s) >= 0.0) % This should definitely be allowed to be zero.  
      % In fact, you should probably even check >= -eps.  
      %    if (psi(incr_p) < min_psi)
      min_psi = psi(incr_s);
      min_psi_idx = incr_s;
    end
    
  end

  % if min_psi is still 1, check to see if psi(1) is actually positive
  if (1 == min_psi_idx) && (psi(1) < 0.0)
    % let's try this again without the psi > 0 constraint
  
    % I think that if you get to here, you should actually just find min(psi.^2)  -jw
    psi2 = psi.^2;
    min_psi_idx = find(psi2==min(psi2),1);
    min_psi = psi(min_psi_idx); 
  
  %   min_psi = psi(1);
  %   for incr_p = 1:size(psi,1)
  %     if (psi(incr_p) < min_psi)
  %       min_psi = psi(incr_p);
  %       min_psi_idx = incr_p;
  %     end
  %   end

  end
  
  if (i_debug > 1)
    fprintf(1, 'Minimum psi found: [%d] %2.2f\n', min_psi_idx, min_psi);
    disp(psi);
  end
  
  % If the min_psi index is not one, swap the edge into the first item
  if (1 ~= min_psi_idx)
    
    % swap psi
    psi_alt = psi(1);
    psi(1) = min_psi;
    psi(min_psi_idx) = psi_alt;
   
    % swap edge vertices
    edges_vertices_alt = edges_vertices(1,:);
    edges_vertices(1,:) = edges_vertices(min_psi_idx,:);
    edges_vertices(min_psi_idx,:) = edges_vertices_alt;
  
  end
  
  
  % build Gn, Gf, E, and b in here
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for incr_s = 1:num_subcontacts(incr_m)
    
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
   
        %      rxd = this_r(1) * this_tangent(2) - this_r(2) * this_tangent(1);
        %      rxd_neg = this_r(1) * -this_tangent(2) - this_r(2) * -this_tangent(1);
        %      Gf_i1 = [[this_tangent; rxd] / norm([this_tangent; rxd]), ...
        %              [-this_tangent; rxd_neg] / norm([-this_tangent; rxd_neg])];
        Gf_i1 = [[this_tangent] / norm([this_tangent]), ...
                [-this_tangent] / norm([-this_tangent])];
        
        Gf(:,fricdirs * incr_s - (fricdirs - 1):fricdirs * incr_s) = Gf_i1;
         
        % FIXME: check that Gf is built only for faces in Gn
        
        
        
        % Build E
        E(fricdirs*incr_s-(fricdirs-1):fricdirs*incr_s,incr_s) = ones(fricdirs,1);
    
 
        
    end
 
    % Build Gn
    %%%%%%%%%%
%      rxn = this_r(1) * this_normal(2) - this_r(2) * this_normal(1);
    %    Gn_i = [-this_normal; rxn] / norm([-this_normal; rxn]);
    Gn_i = -this_normal; % no moment of inertia/rotation

    % (CDA constraints)
    if (incr_s > 1)
      % Build Ga (auxiliary contact wrench)
    
      GaT(incr_GaT,:) = Gn(:,incr_Gn-1)' - Gn_i';
  
      % Build the bottom portions of b
      b(2 * num_active_bodies + num_manifolds + fricdirs*num_contacts + incr_GaT) = (psi(1) - psi(incr_s)) / V.h;
    
      incr_GaT = incr_GaT + 1;
  
    else
      % build Gn (normal contact wrench)
      Gn(:,incr_Gn) = Gn_i; % Gn should only have the active (minimum) edge
      incr_Gn = incr_Gn + 1;
    
      % Build the bottom portions of b
      b(2 * num_active_bodies + incr_m) = psi(incr_s) / V.h;
      
    end
  
    
  end % for incr_s = 1:num_subcontacts
    
    % FIXME: U, and maybe E are the wrong size
    if (V.DEFINE_FRICTION)
        % Build U (assume uniform coefficient of friction)
        U(incr_m,incr_m) = V.mu;
    end
 
  % Build masking matrices (E1, E2)
  this_E1 = ones(1, num_subcontacts(incr_m) - 1);
  this_E2 = tril(ones(num_subcontacts(incr_m)-1));
    
    
  incr_Ecol_end = incr_Ecol + num_subcontacts(incr_m) - 2;
    
  if (i_debug > 4)
    fprintf(1, 'incr_Ecol = %d, incr_Ecol_end = %d\n', incr_Ecol, incr_Ecol_end);
    fprintf(1, 'Size of this_E1: %d x %d\n', size(this_E1,1), size(this_E1,2));
    fprintf(1, 'Size of this_E2: %d x %d\n', size(this_E2,1), size(this_E2,2));
  end
    
  E1(incr_m, incr_Ecol:incr_Ecol_end) = this_E1;
  E2(incr_Ecol:incr_Ecol_end, incr_Ecol:incr_Ecol_end) = this_E2; 
  incr_Ecol = incr_Ecol_end + 1;
  
end % for incr_m = 1:num_manifolds


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Construct main matrix A
% (modified from 4.15 / page 83 in Binh's thesis)
% (NOTE: 4.15 is the hueristic method, not the general form.)

% fprintf(1, 'Size of %s: %d x %d\n', 'MM', size(MM,1), size(MM,2));
% 
% fprintf(1, 'Size of %s: %d x %d\n', 'Gn', size(Gn,1), size(Gn,2));
% fprintf(1, 'Size of %s: %d x %d\n', 'Gf', size(Gf,1), size(Gf,2));
% fprintf(1, 'Size of %s: %d x %d\n', 'GaT', size(GaT,1), size(GaT,2));
% 
% fprintf(1, 'Size of %s: %d x %d\n', 'E', size(E,1), size(E,2));
% fprintf(1, 'Size of %s: %d x %d\n', 'E1', size(E1,1), size(E1,2));
% fprintf(1, 'Size of %s: %d x %d\n', 'E2', size(E2,1), size(E2,2));
% 
% fprintf(1, 'Size of %s: %d x %d\n', 'U', size(U,1), size(U,2));

% smallA = [-MM, Gn, Gf, zeros(size(MM,1), size(E1,2) + size(E,2))];

if (i_debug > 2)
  disp('MM = ');
  disp(MM);

  disp('Gn = ');
  disp(Gn);

  disp('Gf = ');
  disp(Gf);

  disp('GaT = ');
  disp(GaT);
  

%disp('E = ');
%disp(E);

  disp('E1 = ');
  disp(E1);

  disp('E2 = ');
  disp(E2);

end

big = 1e20;

if (V.DEFINE_FRICTION)
    
    % define zero padding matrices
    z4 = zeros(size(MM,1), size(E1,2) + size(E,2));
    z5 = zeros(size(Gn,2), size(Gn,2) + size(Gf,2));
    z6 = zeros(size(Gn,2), size(E,2));
    z7 = zeros(size(Gf,2), size(Gn,2) + size(Gf,2) + size(E1,2));
    z8 = zeros(size(GaT,1), size(Gn,2) + size(Gf,2)); 
    z9 = zeros(size(GaT,1), size(E,2));
    z10 = zeros(size(U,1), size(MM,2));
    z11 = zeros(size(U,1), size(E1,2) + size(E,2));

    A = [-MM, Gn, Gf, z4; ...
        Gn', z5, E1, z6; ...
        Gf', z7, E; ...
        GaT, z8, E2, z9; ...
        z10, U, -E', z11];
else
    A = [-MM, Gn, z1; ...
         Gn', z2, E1; ...
         GaT, z3, E2];
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
%  t_solver = cputime;
t_start = tic;
[z, ~] = pathlcp(A, b, l, u, z0);
%  t_solver = cputime - t_solver;
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

