%function [x err iter flag convergence msg] = psor(A, b, x0, lambda, max_iter, tol_rel, tol_abs, profile)
function solution = psor(obj)
% Copyright 2011, Kenny Erleben, DIKU
% Modified by Ying  2013
% Just a list of human readable text strings to convert the flag return
% code into something readable by writing msg(flag) onto the screen.
% msg = {'preprocessing';  % flag = 1
%   'iterating';      % flag = 2
%   'relative';       % flag = 3
%   'absolute';       % flag = 4
%   'stagnation';     % flag = 5
%   'local minima';   % flag = 6
%   'nondescent';     % flag = 7
%   'maxlimit'        % flag = 8
%   };
% 
% if nargin < 2
%   error('Too few arguments');
% end
% 
% N    = length(b); % Number of variables
% flag = 1;
% 
% %--- Setup meaningfull default values -------------------------------------
% if nargin<3
%   x0 = zeros(N,1);
% end
% if nargin<4
   lambda = 1.0;
% end
% if nargin<5
%   max_iter = floor(N/2);
% end
% if nargin<6
%   tol_rel = 0.0001;
% end
% if nargin<7
%   tol_abs = 10*eps; % Order of 10th of numerical precision seems okay
% end
% if nargin<8
   profile = false;
% end
% 
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
nd = obj.dynamics.nd;

ErrorMetric = obj.solver.errmetric;
TuneParams = obj.solver.TuneParams;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
alpha = TuneParams.alpha;

tol_rel = tol;
tol_abs = tol;

lambda   = min(2, max(0,lambda));

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
solution.stick_or_slide = zeros( max_iter, nc);
solution.z = zeros(size(A, 2), max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);

s = zeros(nc, 1);
pf = zeros(nc*nd, 1);
pn = zeros(nc, 1);
solution = updateSolutionData(solution, ErrorMetric,  iter, A, x0, b, s, U, pn, PSI, pf, nd, Gf, NU);
iter = iter + 1;

while iter <= max_iter
  
  dx = 0;
  for i=1:N
    old_xi = x(i);
    ri     = b(i) + A(i,:)*x;
    Aii    = A(i,i) + 1;
    
    x(i) = max( 0, old_xi - lambda*(ri / Aii) );
    dx   = max(dx, abs(x(i) - old_xi));
  end
  
  old_err = err;
    
  y   = abs( A*x + b );   % Abs is used to fix problem with negative y-values.
  err = x'*y;
  
  if profile
    convergence(iter) = err;
  end
  
  
  pn = x(1:nc);
  pf = x(nc+1 : nc+nc*nd);
  s = x(length(x)-nc+1: end);
  %NU_ellp1 = NU + MinvGn*pn + MinvGf*pf + M\FX*h;
  solution = updateSolutionData(solution, ErrorMetric, iter, A, x, b, s, U, pn, PSI, pf, nd, Gf, NU);
  
  
  if  solution.total_error(iter) < tol
      break;
  end
  % Relative stopping criteria
%   if (abs(err - old_err) / abs(old_err)) < tol_rel  
%     flag = 3;
%     break;
%   end
%   
%   % Absolute stopping criteria
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


if iter>=max_iter
  flag = 8;
  iter = iter - 1;
end

end
