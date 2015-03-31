%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mlcp_fixed_point.m 
%
% MCP solver with prox function formulation with fixed point iteration 
% This is  the strict PGS iterator
function solution = mlcp_strict_pgs( sim )
% matrices used  
Minv = sim.dynamics.Minv;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf; 
h = sim.dynamics.h;

FX = sim.dynamics.Forces;
MinvPext = Minv* FX*h;
PSI = sim.contacts.PSI;
NU = sim.dynamics.Vel;
U = sim.dynamics.U;
E = sim.dynamics.E; 
nd = sim.dynamics.nd;
nc = length(PSI);

ErrorMetric = sim.solver.errmetric;
 
TuneParams = sim.solver.TuneParams;
alpha  = TuneParams.alpha;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
r = TuneParams.r;
rn = r;  rf = r;  rs = r;

% LCP_FIXED_POINT 
% This is the prox projection onto the positive plane using the pyramid 
% iteration with the Newton-Euler equation to make this a mixed LCP problem
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;

A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
      Gf'*MinvGn   Gf'*MinvGf  E
      U            -E'         zeros(nc)];
 
% rn = factor / eigs(Gn' * MinvGn, 1);
% rf = factor / eigs(Gf' * MinvGf, 1);
% temp = 1/2 * (eigs(Gn' * MinvGn, 1) + eigs(Gf' * MinvGf, 1));
% rs = factor / temp;
% parameters to be tuned
 
pn = zeros(nc, 1);
pn_ellp1 = zeros(nc, 1);
pf = zeros(nc*nd, 1);
pf_ellp1 = zeros(nc*nd, 1);
s_ellp1 = zeros(nc, 1); 
s  = zeros(nc, 1);  
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
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc); 

err = Inf;
%% converge the normal force and the frictional force
% converge the normal force first
for iter = 1 : max_iter   
    % Since M has so nice property, we will update the velocity directly
    new_NU = NU + MinvGn*pn + MinvGf*pf + Minv*FX*h;  % update NU_ellp1, but the base is always NU;
    for CT = 1 : nc
        old_pn = pn(CT, 1);
        rho_n = Gn(:, CT)' * new_NU + PSI(CT, 1) / h;
        new_pn = project(old_pn - rn * (rho_n + alpha * old_pn));
        pn(CT, 1) = new_pn;
    end 
    % Iterate over the frictional force 
    for i = 1 : nc*nd
        old_pf = pf(i, 1);
        rho_f = Gf(:, i)' * new_NU + E(i, :) * s;
        new_pf = project(old_pf - rf * (rho_f + alpha * old_pf));
        pf(i, 1) = new_pf;
    end
    for j = 1 : nc
        old_s = s(j, 1);
        rho_s = U(j, j) * pn - E(:, j)' * pf;
        new_s = project(old_s - rs * (rho_s + alpha * old_s));
        s(j, 1) = new_s;
    end
    z = [pn; pf; s]; 
    solution = updateSolutionData(solution, ErrorMetric, iter, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU);
    old_err = err;
    err = solution.total_error(iter);
    if err < tol  && abs(err - old_err) / abs(old_err) < tol
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

