function solution = LCP_strict_PGS( sim )
%LCP_STRICT_PGS use the LCP model, and apply a strict PGS metric to solve
%the problem in a fixed-point iteration way. 

% matrices used  
Minv = sim.dynamics.Minv;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf; 
h = sim.dynamics.h;

FX = sim.dynamics.Forces;
PSI = sim.contacts.PSI;
NU = sim.dynamics.Vel;
U = sim.dynamics.U;
E = sim.dynamics.E; 
nd = sim.dynamics.nd;
nc = length(PSI);

ErrorMetric = sim.solver.errmetric;
 
TuneParams = sim.solver.TuneParams;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
alpha = TuneParams.alpha;

r = TuneParams.r;
rn = r;  rf = r;  rs = r;

%% This part needs to be removed later after test the particle problem
% %NU = [1; 0; -1];
% m = 1;
% NU = [1; 0; -1];
% PSI = 0.15;
% h = 0.1;
% %g = sim.dynamics.Forces(3);  % g is already negative
% g = -5;
% FX = [0; 0; m*g];    % m=1 
% Minv = 1/m*[1 0 0; 0 1 0; 0 0 1];
% U = 0.5 * eye(size(U)); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pn = zeros(nc, 1);
pn_ellp1 = zeros(nc, 1);
pf = zeros(nc*nd, 1);
pf_ellp1 = zeros(nc*nd, 1);
s_ellp1 = zeros(nc, 1); 
s  = zeros(nc, 1);  
MinvPext = Minv* FX*h;
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;

Dnn = Gn' * MinvGn;
Dnf = Gn' * MinvGf;
Dfn = Gf' * MinvGn;
Dff = Gf' * MinvGf;

A = [ Dnn   Dnf   zeros(nc)
      Dfn   Dff      E
      U     -E'   zeros(nc)];
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc);   
solution  = struct();
 
solution.normFBerror = zeros(max_iter, 1);
solution.fricFBerror = zeros(max_iter, 1);
solution.total_error = zeros(max_iter , 1);
solution.normal_error = zeros(max_iter , 1); 
solution.friction_error = zeros(max_iter , 1);
solution.stick_or_slide = zeros(max_iter, 1);
solution.z = zeros(size(A, 2), max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);
 
z = [pn_ellp1; pf_ellp1; s_ellp1];                                                                                                                                                                                                            
%err = Inf;
%% converge the normal force and the frictional force
% converge the normal force first
for iter = 1 : max_iter   
    % Iterate on the normal forces
     for CT = 1 : nc
        old_pn = pn(CT, 1);
        rho_n  = Dnn(CT, :)*pn + Dnf(CT, :) * pf + PSI(CT, 1) / h + Gn(:, CT)' * (NU + MinvPext);
        new_pn = project(old_pn - rn * (rho_n + alpha * old_pn));
        pn(CT, 1) = new_pn;  % update current pn(CT) to use for the next update on pn(CT+1)
     end
       
   
    % Iterate on the frictional forces
    for i = 1 : nc*nd
        old_pf = pf(i, 1);
        rho_f  = Dfn(i, :) * pn + Dff(i, :) * pf + E(i, :) * s + Gf(:, i)' * (NU  + MinvPext);
        new_pf = project(old_pf - rf *(rho_f + alpha * old_pf));
        pf(i, 1) = new_pf;  % PGS way of converge, update immediately after finish an iteration
     end
    
    % Iterate on the sliding speed, which should be always >=0
    for j = 1 : nc
        old_s = s(j, 1);
        rho_s =  U(j, :) * pn  - E(:, j)' * pf;
        new_s = project(old_s - rs * (rho_s + alpha * old_s));
        s(j, 1) = new_s;
     end
    z = [pn; pf; s];
    solution = updateSolutionData(solution, ErrorMetric, iter, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU);
    %tot_err = sum(abs(kanzow(A*z+b, z, 0.7)));
    %tot_err = norm(kanzow(A*z+b, z, 0.7));
    %tot_err = pn_err + pf_err + s_err;
    tot_err = solution.total_error(iter);
    if tot_err < tol 
        break;
    end
end
 
end
 
function new = project(old)
new = max(old, zeros(size(old)));
end
 
function b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc)
b = [ Gn'*(NU + MinvPext) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
      Gf'*(NU + MinvPext);
      zeros(nc,1) ];    
end

function output = kanzow(x, y, lambda)
fischer = (x.^2 + y.^2).^0.5 - x - y;
output = lambda * fischer + (1 - lambda) * (max(x, 0) .* max(y, 0));
end

 
