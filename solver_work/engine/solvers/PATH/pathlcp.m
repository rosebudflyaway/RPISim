function [solution] = pathlcp(sim)
% pathlcp(M,q,l,u,z,A,b,t,mu)
%
% Solve the standard linear complementarity problem using PATH:
%     z >= 0, Mz + q >= 0, z'*(Mz + q) = 0
%
% Required input:
%     M(n,n)  - matrix
%     q(n)    - vector
% 
% Output:
%     z(n)    - solution
%     mu(m)   - multipliers (if polyhedral constraints are present)
%
% Optional input:
%     l(n)    - lower bounds                       default: zero
%     u(n)    - upper bounds                       default: infinity
%     z(n)    - starting point                     default: zero
%     A(m,n)  - polyhedral constraint matrix       default: empty
%     b(m)    - polyhedral right-hand side         default: empty
%     t(m)    - type of polyhedral constraint      default: 1
%                  < 0: less than or equal
%                    0: equation
%                  > 0: greater than or equal
%     mu(m)   - starting value for multipliers     default: zero
%
% The optional lower and upper bounds are used to define a linear mixed 
% complementarity problem (box constrained variational inequality).
%       l <= z <= u
%       where l_i < z_i < u_i  => (Mz + q)_i = 0
%             l_i = z          => (Mz + q)_i >= 0
%             u_i = z          => (Mz + q)_i <= 0
% 
% The optional constraints are used to define a polyhedrally constrained
% variational inequality.  These are transformed internally to a standard
% mixed complementarity problem.  The polyhedral constraints are of the
% form
%       Ax ? b
% where ? can be <=, =, or >= depending on the type specified for each
% constraint.

Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf;
NU = sim.bodies.velocities;
h = sim.dynamics.h;
nc = length(sim.contacts.PSI);
nd = sim.num_fricdirs;
ni = 1;
M = sim.dynamics.A_MCP;
q = sim.dynamics.b_MCP;

nb = length(sim.bodies.velocities) / 6;
nc = length(sim.contacts.PSI);
nd = sim.dynamics.nd;
U = sim.dynamics.U;
PSI = sim.contacts.PSI;

l = sim.solution.l;
u = sim.solution.u;
z = sim.dynamics.z0_MCP;
     
ErrorMetric = sim.solver.errmetric;
solution.iterations = 0;
solution.total_error = zeros(ni,2);
solution.normal_error = zeros(ni,2);
solution.friction_error = zeros(ni,2);
solution.stick_or_slip = zeros(ni, 2);
solution.z = zeros(length(q), ni);
solution.solver_name = 'PATH';
solution.direction_error = zeros(ni, 2);
solution.copositive_normal_error = zeros(ni, 2);
solution.copositive_friction_error = zeros(ni, 2);
solution.normal_neg_error = zeros(ni, 2);
solution.fric_neg_error = zeros(ni, 2);
solution.normFBerror = zeros(ni, 2);
solution.fricFBerror = zeros(ni, 2);

Big = 1e20;


%     
% if (nargin < 2)
%   error('two input arguments required for lcp(M, q)');
% end

if (~issparse(M))
  M = sparse(M);	% Make sure M is sparse
end
q = full(q(:)); 	% Make sure q is a column vector

[mm,mn] = size(M);	% Get the size of the inputs
n = length(q);

if (mm ~= mn || mm ~= n) 
  error('dimensions of M and q must match');
end

if (n == 0)
  error('empty model');
end

% if (nargin < 3 || isempty(l))
%   l = zeros(n,1);
% end
% 
% if (nargin < 4 || isempty(u))
%   u = Big*ones(n,1);
% end
% 
% if (nargin < 5 || isempty(z))
%   z = zeros(n,1);
% end

z = full(z(:)); l = full(l(:)); u = full(u(:));
if (length(z) ~= n || length(l) ~= n || length(u) ~= n)
  error('Input arguments are of incompatible sizes');
end

l = max(l,-Big*ones(n,1));
u = min(u,Big*ones(n,1));
z = min(max(z,l),u);

m = 0;
if (nargin > 5)
  if (nargin < 7)
    error('Polyhedral constraints require A and b');
  end

  if (~issparse(A))
    A = sparse(A);
  end
  b = full(b(:));

  m = length(b);

  if (m > 0)

    [am, an] = size(A);

    if (am ~= m || an ~= n)
      error('Polyhedral constraints of incompatible sizes');
    end

    if (nargin < 8 || isempty(t))
      t = ones(m,1);
    end

    if (nargin < 9 || isempty(mu))
      mu = zeros(m,1);
    end

    t = full(t(:)); mu = full(mu(:));
    if (length(t) ~= m || length(mu) ~= m)
      error('Polyhedral input arguments are of incompatible sizes');
    end

    l_p = -Big*ones(m,1);
    u_p =  Big*ones(m,1);

    idx = find(t > 0);
    if (~isempty(idx))
      l_p(idx) = zeros(length(idx),1);
    end

    idx = find(t < 0);
    if (~isempty(idx))
      u_p(idx) = zeros(length(idx),1);
    end

    mu = min(max(mu,l_p),u_p);

    M = [M -A'; A sparse(m,m)];
    q = [q; -b];

    z = [z; mu];
    l = [l; l_p];
    u = [u; u_p];
  else
    if (nargin >= 9 && ~isempty(mu))
      error('No polyhedral constraints -- multipliers set.');
    end

    if (nargin >= 8 && ~isempty(t))
      error('No polyhedral constraints -- equation types set.');
    end
  end
end

idx = find(l > u, 1);
if ~isempty(idx)
  error('Bounds infeasible.');
end

nnzJ = nnz(M);

[status, ttime] = lcppath(n+m, nnzJ, z, l, u, M, q);

if (status ~= 1) 
  status;
  error('Path fails to solve problem');
end

mu = [];
if (m > 0)
  mu = z(n+1:n+m);
  z = z(1:n);
end

    pn = z(6*nb+1 : 6*nb+nc, 1);  
    pf = z(6*nb+nc+1 : 6*nb+nc+nc*nd, 1);
    s = z(length(z)-nc+1 : end);
    iter = 1;
    Vel = z(1:6*nb);
    solution.z = z;
    solution.NUnew = Vel;
    solution = updateSolutionData(solution, ErrorMetric, iter, M, z, q, s, U, pn, PSI, pf, nd, Gf, Vel);
    solution.iterations = 2;
    solution.total_error(2) = z' * (M * z + q);
return;

