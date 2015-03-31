%function [z, iter, err, solution] = lemke(M,q,z0,varargin)
function solution = lemke( sim ) 
% syntax: [z,err] = lemke(M,q,z0)
% LEMKE    Solves linear complementarity problems (LCPs).
% An LCP solves
%   Mz+q >=0, z>=0, z'(Mz+q)=0.
% The input z0 defines a starting basis; it can be either
% an initial guess of the solution or a vector of zeros and ones 
% with ones representing those z(i) thought to be non-zero in the
% solution.  For example, passing z=[1.5;0;2.2] has the same 
% effect as passing z=[1;0;1]. 
% If z0 is omitted the origin is used as a starting basis.
% ERR returns an error condition:
%   0: Solution found
%   1: Maximum iterations exceeded
%   2: Unbounded ray termination
% If NARGOUT==1, a warning message is displayed instead.
%
% ALGORITHM
%   Uses a modified Lemke's algorithm (complementary pivoting)
%   with a covering ray of ones.  The algorithm is modified to
%   allow a user defined initial basis.

M = sim.dynamics.A_LCP;
q = sim.dynamics.b_LCP;
%sim.dynamics
NU = sim.dynamics.Vel;
FX = sim.dynamics.Forces;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf;
Mass = sim.dynamics.M;
Minv = sim.dynamics.Minv;
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;
h = sim.dynamics.h;
z0 = zeros(length(q),1); 

nc = length(sim.contacts.PSI);
nd = sim.num_fricdirs; 
U = sim.dynamics.U; 
psi_n = sim.contacts.PSI; 
ni = min([10000  25*length(q)]);   % Init as the max # of iterations

ErrorMetric = sim.solver.errmetric;

solution.iterations = 0;
solution.normFBerror = zeros(ni, 1);
solution.fricFBerror = zeros(ni, 1);
solution.total_error = zeros(ni,1);
solution.normal_error = zeros(ni,1);
solution.friction_error = zeros(ni,1);
solution.stick_or_slip = zeros(ni, 1);
solution.z = zeros(length(q), ni);
solution.solver_name = 'Lemke';
solution.direction_error = zeros(ni, 1);
solution.copositive_normal_error = zeros(ni, 1);
solution.copositive_friction_error = zeros(ni, 1);
solution.normal_neg_error = zeros(ni, 1);
solution.fric_neg_error = zeros(ni, 1);

n = length(q);
zer_tol = 1e-5;

%piv_tol = 1e-8;
if isfield(sim.solver,'tolerance') && sim.solver.tolerance > 0
    piv_tol = sim.solver.tolerance;
else
    piv_tol = 1e-8;    % The tolerance of the error
end

iter = 1;
s = zeros(nc, 1);
pf = zeros(nc*nd, 1);
pn = zeros(nc, 1);
 
solution = updateSolutionData(solution, ErrorMetric, iter, M, z0, q, s, U, pn,  psi_n, pf, nd, Gf, NU);
maxiter = min([10000  25*n]);
err=0;

% Trivial solution exists
if all(q >= 0.)
  z=zeros(n,1); return;
end
z = zeros(2*n,1);
j = zeros(n,1);

% Determine initial basis
if nargin<3 
  bas=(n+1:2*n)'; 
  B = -speye(n);
else
  bas=[find(z0>0);n+find(z0<=0)];
  B = [sparse(M) -speye(n)];
  B = B(:,bas);
end

% Determine initial values
x=-(B\q);

% Check if initial basis provides solution
if all(x>=0) 
  z(bas)=x; z=z(1:n); 
  return 
end

t = 2*n+1;      % Artificial variable
entering=t;     % is the first entering variable

% Determine initial leaving variable
[tval,lvindex]=max(-x);
leaving=bas(lvindex);

bas(lvindex)=t;       % pivot in the artificial variable
x=x+tval;
x(lvindex)=tval;
B(:,lvindex)=-B*ones(n,1);

% Main iterations begin here
for iter=2:maxiter
  % Check if done; if not, get new entering variable
  if (leaving == t) break;
  elseif (leaving <= n)
    entering = n+leaving;
    Be = sparse(leaving,1,-1.0,n,1);
  else
    entering = leaving-n;
    Be = M(:,entering);
  end
  d = B\Be;

  % Find new leaving variable
  j=find(d>piv_tol);                  % indices of d>0
  if isempty(j)                       % no new pivots - ray termination  
    err=2; 
    break
  end
  theta=min((x(j)+zer_tol)./d(j));    % minimal ratios, d>0
  j=j(find((x(j)./d(j))<=theta));     % indices of minimal ratios, d>0
  lvindex=find(bas(j)==t);            % check if artificial among these
  if ~isempty(lvindex)                % Always use artifical if possible
    lvindex=j(lvindex);
  else                                % otherwise pick among set of max d 
    theta=max(d(j));        
    lvindex=find(d(j)==theta);
    lvindex=j(ceil(length(lvindex)*rand));  % if multiple choose randomly
  end
  leaving=bas(lvindex); 

  % Perform pivot
  ratio=x(lvindex)./d(lvindex);
  x = x - ratio*d;
  x(lvindex) = ratio;
  B(:,lvindex) = Be;
  bas(lvindex) = entering;
  s = z(n-nc+1:n);
  pn = z(1:nc);
  pf = z(nc+1:n-nc);
  clear z; z(bas) = x; z=z'; z = z(1:n); 
  NUnew = NU + MinvGn*pn + MinvGf*pf + Mass\FX*h;
  solution = updateSolutionData(solution, ErrorMetric, iter, M, z, q, s, U, pn, psi_n, pf, nd, Gf, NUnew);
end
solution.NUnew = NUnew;% end of iterations
if iter>=maxiter && leaving~=t err=1; end
clear z; z(bas) = x; z = z(1:n); 

% Display warning messages if no error code is returned
if nargout<2 && err(1)~=0
  s='Warning: solution not found - ';
  if err(1)==2
    disp([s 'Unbounded ray']);
  elseif err(1)==1
    disp([f 'Iterations exceeded limit']);
  end
end

end


