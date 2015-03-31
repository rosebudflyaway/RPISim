%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mlcp_fixed_point.m 
%
% This is solve the big A matrix as a whole
% 1---------------solve pn 
% 2---------------solve pf
% 3---------------solve s
function solution = LCP_fixed_point( sim )
Minv = sim.dynamics.Minv;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf; 
h = sim.dynamics.h;
FX = sim.dynamics.Forces;
MinvPext = Minv * FX * h;
PSI = sim.contacts.PSI;
PSI = 0.15;
NU = sim.dynamics.Vel;
U = sim.dynamics.U;
E = sim.dynamics.E; 
nc = length(PSI);
nd = sim.dynamics.nd;
position = sim.dynamics.position;

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

ErrorMetric = sim.solver.errmetric;

TuneParams = sim.solver.TuneParams;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
r = TuneParams.r;
% rn = r;  rf = r; rs = r;

pn       = zeros(nc, 1);
pf       = zeros(nc*nd, 1);
s        = zeros(nc, 1);  
 
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
 
z = [pn; pf; s];
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc); 
err = Inf;
% Make sure NU is constant inside the loop. No update on NU 
%% converge the normal force and the frictional force
 
for iter = 1 : max_iter 
    z_ellp1 = z - r * (A*z + b);
    z_ellp1(z_ellp1<0) = 0;
    z  = z_ellp1;

    pn = z(1:nc, 1);
    pf = z(nc+1:nc*(nd+1), 1);
    s = z(length(z)-nc:end, 1);
    
    solution = updateSolutionData(solution, ErrorMetric, iter, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU);
    err     = solution.total_error(iter);
    if err < tol  %&& abs(err - old_err) / abs(old_err) < tol
        break;
    end
end
NU_ellp1 = NU + MinvGn * pn + MinvGf * pf + MinvPext;
solution.z = NU_ellp1;

end
 
function b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc)
b = [ Gn'*(NU + MinvPext) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
      Gf'*(NU + MinvPext);
      zeros(nc,1) ];    
end

