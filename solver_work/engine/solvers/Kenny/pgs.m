%function [x err iter flag convergence msg] = pgs(A, b, x0, max_iter, tol_rel, tol_abs, profile)
function solution = pgs(obj)
% Copyright 2011, Kenny Erleben, DIKU
% modified  to use the standard solver interface by Ying Lu. 
A = obj.dynamics.A_LCP;
b = obj.dynamics.b_LCP;
x0 = obj.dynamics.z0_LCP;

N = length(b);
 
profile = false;
U = obj.dynamics.U;
Gf = obj.dynamics.Gf;
NU = obj.dynamics.Vel;
M = obj.dynamics.M;
Minv = obj.dynamics.Minv;
Gn = obj.dynamics.Gn;
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;
h = obj.dynamics.h;
FX = obj.dynamics.Forces;
PSI = obj.contacts.PSI;
nc = length(PSI);
nd = size(Gf, 2) / nc;

ErrorMetric = obj.solver.errmetric;
TuneParams = obj.solver.TuneParams;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
alpha = TuneParams.alpha;
  

%--- Setup values needed while iterating ------------------------------------
convergence = zeros(max_iter,1); % Used when profiling to measure the convergence rate
err         = Inf;               % Current error measure
x           = x0;                % Current iterate
iter        = 1;                 % Iteration counter
flag        = 2;
solution = struct();
solution.normFBerror = zeros(max_iter, 1);
solution.fricFBerror = zeros(max_iter, 1);
solution.total_error = zeros(max_iter, 1);
solution.normal_error = zeros(max_iter, 1); 
solution.friction_error = zeros(max_iter, 1) ;
solution.stick_or_slide = zeros( max_iter, 1);
solution.z = zeros(size(A, 2), max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);
% Make sure the initial error is the same 
pn = zeros(nc, 1);
pf = zeros(nc*nd, 1);
s = zeros(nc, 1);
solution = updateSolutionData(solution, ErrorMetric, iter, A, x, b, s, U, pn, PSI, pf, nd, Gf, NU);
iter = iter + 1;

% deal with A for the different properties
% [M1, ~] = lu(A);
% M = M1 + diag(diag(A));
% N = A - M;
while iter <= max_iter
  dx = 0;
  for i=1:N
    old_xi = x(i);
    ri     = b(i) + A(i,:)*x;
    Aii    = A(i,i) + 1;
    x(i) = max( 0, old_xi - 0.9 * (ri / Aii) );
    dx = max(dx, abs(x(i) - old_xi)); % projection
  end
  old_err = err;
    
  % y   = abs( A*x + b );   % Abs is used to fix problem with negative y-values.
  % err = x'*y;
  pn = x(1:nc);
  pf = x(nc+1 : nc+nc*nd);
  s = x(length(x)-nc+1: end);
  NU_ellp1 = NU + MinvGn*pn + MinvGf*pf + Minv*FX*h;
  solution = updateSolutionData(solution, ErrorMetric, iter, A, x, b, s, U, pn, PSI, pf, nd, Gf, NU_ellp1);
  
  if profile
    convergence(iter) = err;
  end
  err = x' * (A * x + b);
  
%   
  % Relative stopping criteria
  if (abs(err - old_err) / abs(old_err)) < tol  &&   err < tol    && dx < tol
    break;
  end
  
  % Absolute stopping criteria
%   if err < tol_abs   
%     flag = 4;
%     break;
%   end
%   
%   % Stagnation testing
%   if dx < eps
%     flag = 5;
%     break;
%   end
  
  iter = iter + 1;
end

% if iter>=max_iter
%   flag = 8;
%   iter = iter - 1;
% end

end
