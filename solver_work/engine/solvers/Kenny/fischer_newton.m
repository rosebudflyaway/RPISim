%function [ x err iter flag convergence msg] = fischer_newton( A, b, x0, max_iter, tol_rel, tol_abs, solver, profile )
function solution = fischer_newton(obj)
% Copyright 2011, Kenny Erleben, DIKU
% modified by Ying  2013 
% the updateSolutionData is called at the end of the solver, which is
% inside the solver iteration, so we could easily move that to the
% beginning of the solver, inside the iterations, which is exactly the same
msg = {'preprocessing';  % flag = 1
       'iterating';      % flag = 2
       'relative';       % flag = 3
       'absolute';       % flag = 4
       'stagnation';     % flag = 5
       'local minima';   % flag = 6
       'nondescent';     % flag = 7
       'maxlimit'        % flag = 8
      };

% Input to the fischer_newton solver
A = obj.dynamics.A_LCP;
b = obj.dynamics.b_LCP;
x0 = obj.dynamics.z0_LCP;
solver = 'penalized';   % only used for FB function

TuneParams = obj.solver.TuneParams;
ErrorMetric = obj.solver.errmetric;
max_iter = TuneParams.max_iter;
tol  = TuneParams.tol;
tol_rel = tol;
tol_abs = tol;
% For the lambda used in the objective function, we will use the TuneParams
% to define, the lambda used in the ErrorMetric is specially for the error
lambda1 = TuneParams.lambda1;
lambda2 = TuneParams.lambda2;
meritFunc = TuneParams.objectiveFunction;
meritFuncHandle = str2func(meritFunc);


% Variables used in the updateSolutionData function
N = length(b);
U = obj.dynamics.U;
Gf = obj.dynamics.Gf;
NU = obj.dynamics.Vel;
Minv = obj.dynamics.Minv;
Gn = obj.dynamics.Gn;
MinvGn = Minv * Gn;
MinvGf = Minv * Gf;
h = obj.dynamics.h;
FX = obj.dynamics.Forces;
PSI = obj.contacts.PSI;
nc = length(obj.contacts.PSI);
nd = obj.dynamics.nd;
 
%--- Make sure all values are valid --------------------------------------- 
%--- Here comes a bunch of magic constants --------------------------------
h_deriva = 1e-7;    % Fixed constant used to evaluate the directional detivative
alpha   = 0.5;     % Step reduction parameter for projected Armijo backtracking line search
beta    = 0.001;   % Sufficent decrease parameter for projected Armijo backtracking line search
gamma   = 1e-20;   % Perturbation values used to fix near singular points in derivative
rho     = eps;     % Descent direction test parameter used to test if the Newton direction does a good enough job.
%--- Setup values need while iterating ------------------------------------
x       = x0;          % Current iterate

%--- Preallocate for the solution struct. 
%--- OUTPUT of this function. 
solution = struct();
solution.normFBerror = zeros(max_iter, 1);
solution.fricFBerror = zeros(max_iter, 1);
solution.total_error = zeros(max_iter, 1);
solution.normal_error = zeros(max_iter, 1); 
solution.friction_error = zeros(max_iter, 1) ;
solution.stick_or_slide = zeros(max_iter, 1);
solution.z = zeros(size(A, 2), max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);
solution.flag = zeros(max_iter, 1);

iter    = 1;           % Iteration counter
flag    = 2;
err     = Inf;

% Shows the convergence of a single simulation 
while (iter <= max_iter)
  y = A*x + b;
  %--- Test all stopping criteria used ------------------------------------
  pn = x(1:nc);
  pf = x(nc+1 : nc+nc*nd);
  s = x(length(x)-nc+1: end);
  %NU_ellp1 = NU + MinvGn*pn + MinvGf*pf + Minv*FX*h;
 
  switch meritFunc
      case 'FB'
          [phi, psi, J, dx] = feval(meritFuncHandle, x, y, solver);
      case 'CCK'
          [phi, psi, J, dx] = feval(meritFuncHandle, x, y, A, lambda1);
      case 'mCCK'
          fricTerm = solution.friction_error(iter);
          Jfric = zeros(length(x), 1);
          Jfric(nc+1 : (nd+1)*nc, 1) = abs(Gf' * NU)';
          [phi, psi, J, dx] = feval(meritFuncHandle, x, y, lambda1, lambda2, fricTerm, Jfric);
  end
  nabla_phi = phi'*J;
  %--- Armijo backtracking combined with a projected line-search ---------
  tau     = 1.0;                  % Current step length
  f_0     = err;
  grad_f  = beta*(nabla_phi*dx);
  x_k     = x;

  while true
    x_k   = max(0,x + dx*tau);
    y_k   = A*x_k + b;  
    % Update again on the phi, psi with x_k, y_k
    switch meritFunc
        case 'FB'
            [phi, psi, J, dx] = feval(meritFuncHandle, x_k, y_k, solver);
        case 'CCK'
            [phi, psi, J, dx] = feval(meritFuncHandle, x_k, y_k, A, lambda1);
        case 'mCCK'
            fricTerm = solution.friction_error(iter);
            Jfric = zeros(length(x_k), 1);
            Jfric(nc+1 : (nd+1)*nc, 1) = abs(pf' * NU);
            [phi, psi, J, dx] = feval(meritFuncHandle, x_k, y_k, A, lambda1, lambda2, fricTerm, Jfric);
    end
    phi_k = phi;
    f_k = psi;   
    % Perform Armijo codition to see if we got a sufficient decrease
    if ( f_k <= f_0 + tau*grad_f)
      break;
    end
    
    % Test if time-step became too small
    if tau*tau < gamma
      break;
    end
    tau = alpha * tau;
  end
  
  % Update iterate with result from Armijo backtracking
  x = x_k;
  % Update the solution
  solution = updateSolutionData( solution, ErrorMetric, iter, A, x, b, s, U, pn, PSI, pf, nd, Gf, NU);
  old_err = err;
  err = solution.total_error(iter);  % This err is calculated in the updateSolutionData. 
  
  % Check the termination criteria
  if (abs(err - old_err) / abs(old_err) < tol_rel)
      flag = 3;
      break;
  end
  if err < tol_abs
      flag = 4;
      break;
  end  
  % Test if the search direction is smaller than numerical precision. That is if it is too close to zero. 
  if max(abs(dx)) < eps
    flag = 5;
    % Rather than just giving up we may just use the gradient direction
    % instead. However, I am lazy here!
    %  dx = nabla_phi'
    break;
  end
  
  % Test if we have dropped into a local minimia if so we are stuck
  
  if norm(nabla_phi) < tol_abs
      flag = 6;
      break;
  end
  
  % Test if our search direction is a 'sufficient' descent direction
  if nabla_phi*dx > -rho*((dx)'*dx)
      flag = 7;
      % Rather than just giving up we may just use the gradient direction 
      % instand.
      %  dx = nabla_phi';
      break;
  end
  
  % Increment the number of iterations
  iter=iter+1;
end

 %fprintf('%12.4f \t %12.4f \n', x, A*x+b);
  

if iter>=max_iter
  flag = 8;
  iter = iter - 1;
end
solution.flag(iter) = flag;
end

function [phi, psi, J, dx] = FB(x, y, solver)
% This function returns the psi, phi function, where psi is the NCP
% function and phi is the objective function; Also return J as jacobian
% matrix to solve for delta x

% Input: 
% x, y ------------------- 0 <= x \perp y <= 0 
% Output: 
% psi -------------------- the objective function (merit function) 
%                 -------- psi = 1/2*(phi')*phi
% phi -------------------- FB function 
% J   -------------------- The Jacobian matrix of the FB function, which is
%                     ---- the C-subdifferential of the FB function. 
gamma = 1e-20;   
N = length(x);
phi = (y.^2 + x.^2).^(0.5) - x - y;
psi = 1/2 * (phi') * phi;
dx = zeros(N,1); % Allocate space for computing the Newton direction

S       = abs(phi)<gamma & abs(x)<gamma;  % Bitmask for singular indices
I       = find(S==0);                     % Bitmask for non-singular indices
restart = min(length(x),10);              % The number of iterations done before GMRES should restart
% The computation of J depends on the subsolver for the subsystem
switch lower(solver)
    case 'random'    % works on full system
        q       = rand(N,1) ./ sqrt(2)  - 1;
        p       = rand(N,1) ./ sqrt(2)  - 1;
        J       = sparse( zeros(N,N) );
        q(I)    = (y(I)./((y(I).^2+x(I).^2).^0.5))-1;
        p(I)    = (x(I)./((y(I).^2+x(I).^2).^0.5))-1;
        J(I,I)  = diag(p(I))*eye(length(I)) + diag(q(I))*A(I,I);
        [dx ~]  = gmres( J, (-phi), restart);
    case 'perturbation'     % works on full system
        px          = x;
        dir         = sign( x );
        dir(dir==0) = 1;
        px(S==1)    = gamma * dir(S==1);
        p = (px./((y.^2+px.^2).^0.5))-1;
        q = ( y./((y.^2+px.^2).^0.5))-1;
        J = diag(p)*eye(N) + diag(q)*A;
        
        % Billups solution actually pick the gradient direction as the descent
        % direction if the Newton system can not be solved. Look in his
        % thesis at page 85.
        %
        % %if rcond(J) < eps
        % %  dx = -J'*phi;          % Too bad we pick gradient direction
        % %end
        %
        %Instead one may use the pseudo inverse if badly conditioned
        %
        if rcond(J) < eps*2
            dx = pinv(J) * (-phi); % badly conditioned
        else
            dx = J \ (-phi);       % well conditioned
        end
        %
        % Or one can hope for the best and try GMRESconv
        %
        %[dx ~]  = gmres( J, (-phi), restart);
        %
    case 'zero'   % works on reduced system would be similar to random-case if all radom values were set to 1
        J       = sparse( zeros(N,N) );
        q       = (y(I)./((y(I).^2+x(I).^2).^0.5))-1;
        p       = (x(I)./((y(I).^2+x(I).^2).^0.5))-1;
        J(I,I)  = diag(p)*eye(length(I)) + diag(q)*A(I,I);
        
        [dx(I) ~] = gmres( J(I,I), (-phi(I)), restart);
        
    case 'approximation'  % works on reduced system assuming zero Jacobian for all bad values
        J       = sparse( zeros(N,N) );
        q       = (y(I)./((y(I).^2+x(I).^2).^0.5))-1;
        p       = (x(I)./((y(I).^2+x(I).^2).^0.5))-1;
        J(I,I)  = diag(p)*eye(length(I)) + diag(q)*A(I,I);
        
        fun       = @(dx) ( fischer( A(I,I)*(x(I)+ dx*h_deriva) + b(I), x(I) + dx*h_deriva) - phi(I) ) / h_deriva;
        [dx(I) ~] = gmres( fun, (-phi(I)), restart);
end
end


function [phi, psi, J, dx] = CCK(x, y, A, lambda1)
N = length(x);
fb =   (y.^2 + x.^2).^(0.5) - x - y;
phi = lambda1*fb + (1 - lambda1) * (max(0, x) .* max(0, y));
psi = 1/2*(phi') * phi;
% Produces an element of V_{k} \in \partial_C \Phi_{\lambda}
% according to algorithm 2.4 of A Penalized Fischer-Burmeister
% Ncp-Function: Theoretical Investigation And Numerical Results
% (see p. 8).
SMALL = 1e-10; % Kanzow uses 1e-8
z = zeros(N,1);
S1 = abs(x) <= SMALL & abs(y) <= SMALL;
S2 = x > SMALL & y > SMALL;
NS = ~(S1 | S2);
Da = zeros(N,1);
Db = zeros(N,1);
%%% Does not really seem to matter whether we use rand or ones!
z(S1) = ones(length(find(S1)),1);  % length(find(S1)) returns the length of the nonzero elements in S1
% z(S1) = 0.5*ones(length(find(S1)),1);
% z(S1) = rand(length(find(S1)),1);
denomS1 = sqrt(z(S1).^2+(A(S1,S1)*z(S1)).^2);
Da(S1) = lambda1*(z(S1)./denomS1-1);
Db(S1) = lambda1*((A(S1,S1)*z(S1))./denomS1-1);

denomS2 = sqrt(x(S2).^2+y(S2).^2);
Da(S2) = lambda1*(x(S2)./denomS2-1)+(1-lambda1)*y(S2);
Db(S2) = lambda1*(y(S2)./denomS2-1)+(1-lambda1)*x(S2);

% For the indicies that not in S1 or S2
denomNS = sqrt(x(NS).^2+y(NS).^2);
Da(NS) = lambda1*(x(NS)./denomNS-1);
Db(NS) = lambda1*(y(NS)./denomNS-1);

% J = Vk
J = diag(Da) + diag(Db)*A;
dx = J \ (-phi);
end

function [phi psi J dx] = mCCK(x, y, A, lambda1, lambda2, fricError, Jfric)
N = length(x);
cck = lambda1 * ((x.^2 + y.^2).^(0.5) - x - y) + (1 - lambda1) * (max(0, x) .* max(0, y)) ;
phi = lambda2 * cck + (1 - lambda2) * fricError;
psi = 1/2* (phi') * phi;
% Produces an element of V_{k} \in \partial_C \Phi_{\lambda}
% according to algorithm 2.4 of A Penalized Fischer-Burmeister
% Ncp-Function: Theoretical Investigation And Numerical Results
% (see p. 8).
SMALL = 1e-10; % Kanzow uses 1e-8
z = zeros(N,1);
S1 = abs(x) <= SMALL & abs(y) <= SMALL;
S2 = x > SMALL & y > SMALL;
NS = ~(S1 | S2);
Da = zeros(N,1);
Db = zeros(N,1);
%%% Does not really seem to matter whether we use rand or ones!
z(S1) = ones(length(find(S1)),1);  % length(find(S1)) returns the length of the nonzero elements in S1
% z(S1) = 0.5*ones(length(find(S1)),1);
% z(S1) = rand(length(find(S1)),1);
denomS1 = sqrt(z(S1).^2+(A(S1,S1)*z(S1)).^2);
Da(S1) = lambda1*(z(S1)./denomS1-1);
Db(S1) = lambda1*((A(S1,S1)*z(S1))./denomS1-1);

denomS2 = sqrt(x(S2).^2+y(S2).^2);
Da(S2) = lambda1*(x(S2)./denomS2-1)+(1-lambda1)*y(S2);
Db(S2) = lambda1*(y(S2)./denomS2-1)+(1-lambda1)*x(S2);

% For the indicies that not in S1 or S2
denomNS = sqrt(x(NS).^2+y(NS).^2);
Da(NS) = lambda1*(x(NS)./denomNS-1);
Db(NS) = lambda1*(y(NS)./denomNS-1);

% J1 is the Jacobian of the CCK part, we have to add J_frict
J1 = diag(Da) + diag(Db)*A;

J_frict = diag(Jfric);
J = lambda2 * J1 + (1 - lambda2) * J_frict;

if rcond(J) < eps*2
    dx = pinv(Vk) * (-phi_l); % badly conditioned
else
    dx = Vk \ (-phi_l);       % well conditioned
end

%         dx = Vk \ (-phi_l);
end 
