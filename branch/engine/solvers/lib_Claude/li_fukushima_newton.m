% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

function [z, w, idx, it, err, epsilon, step, sit] = li_fukushima_newton (M, q,  l, u, maxit, tol, z0)
  % usage:  [z, w, idx, it, err] = li_fukushima_newton (M, q, l, u,
  % maxit, tol, z0)
  %
  % apply smoothed Newton method due to Li and Fukushima, "Smoothing
  % and Quasi-Newton Methods for Mixed Complementarity Problems",
  % Computational Optimization and Applications, 17, 203-230, (2000).
  %
  % The LCP to solve is
  %  M*z + q = w1 - w2
  % with
  % 0 <= z - l  perp w1 >= 0
  % 0 <= u - z  perp w2 >= 0 
  %
  % Lower and upper bounds can be infinite
  %

  % INPUTS:
  % M     : a square matrix (in principle, any P matrix will do)
  % q     : negative RHS vector (see above)
  % l     : vector of lower bounds (can be -Inf)
  % u     : vector of upper bounds (can be  Inf)
  % maxit : maximum number of iterations
  % tol   : tolerance for solution
  % z0    : optional start vector

  % some arbitrary parameters
  % 0 < rho2 < 1 : component-wise decrease criterion: determines gamma
  % 0 < gamma < 1: global decrease criterion from rho2
  % 0 < rho1 <   : line search control
  % 0 < eps0 < 1 : determined from gamma
  % 0 < eta0 <   : relaxation of line search (generates etak = eta0^k)
  % 0 < sigma1   : relaxation of line search
  % 0 < sigma2   : relaxation of line search
  % 0 < tol      : the stopping criterion

  % the index set is defined as follows:
  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;
  
  n = length(q); 
      
  rho2    = 0.3;
  rho1    = 0.5;
  sigma1  = 1E-6;
  sigma2  = 1E-6;
  eta0    = 0.90; 
  gamma = 0.2* min(0.5, rho2)/sqrt(n); 
  done = 0; 
  it = 0;
 
  l_inf = find(any([l==-Inf, l==Inf], 2)); 
  u_inf = find(any([u==-Inf, u==Inf], 2)); 
  l_fin = find(all([l~=-Inf, l~=Inf], 2)); 
  u_fin = find(all([u~=-Inf, u~=Inf], 2));  

  if ( ~ exist('maxit', 'var') )
    maxit = n;
  end%if
  
  if ( ~ exist('z0', 'var') ) 
    z0 = 0 * q; 
  end%if

  if ( ~ exist('tol', 'var') )
    tol = 1E-10;
  end%if
  z = z0; 

  w = M*z + q; 

  
  phi0 = get_phi(l, u, z, w, 0, l_inf, u_inf, l_fin, u_fin); 
  nphi0 = norm(phi0);
  ek = gamma*nphi0/3;
  nphik = norm(get_phi(l, u, z, w, ek, l_inf, u_inf, l_fin, u_fin)); 
  it = 1; 
  err(it) = max(norm(nphi0), eps); 
  epsilon(it)= ek;
  done = (err(it) < tol || it > maxit); 


  % dynamic variables
  etak  = eta0; 

  while ( ~done )
    [a, b] = get_ab(l, u, z, w, ek, l_inf, u_inf, l_fin, u_fin); 
    M1 = diag(a) + diag(b)*M ; 
    p = -M1\phi0; 
    p2 = p'*p;
    lambda = 1;
    eta = 0;
    sigma = sigma1;
    zt = z + lambda*p;
    wt = M*zt + q; 
    rho = rho2; 
    nphikt = norm(get_phi(l, u, zt, wt, ek, l_inf, u_inf, l_fin, u_fin)); 
    % do the line search.
    lit = 0;
    while ( ( nphikt - rho*nphik ) > ( -sigma*lambda*lambda*p2 + eta ) )
      lambda = lambda*rho1 ;
      zt = z + lambda*p; 
      wt = M*zt + q; 
      eta = etak; 
      sigma  = sigma2; 
      rho = 1; 
      nphikt = norm(get_phi(l, u, zt, wt, ek, l_inf, u_inf, l_fin, u_fin));  
      
      if ( 0 && lit > 20 )
	fprintf('bad error k=%d err = %1.1E %.1E %0.1E %0.1E', ...
		     it, err(it), ...
		     lambda,...
		     ( nphikt - rho*nphik ) - ( -sigma*lambda*lambda*p2 + eta ), etak);
         break;
      end%if
      lit = lit + 1;
    end%while
    % update dynamic parameters.
    etak = eta*eta0; 
    z = zt;
    w = wt; 
    phi0 = get_phi(l, u, z, w, 0, l_inf, u_inf, l_fin, u_fin); 
    nphi0 = norm(phi0);
    if ( ek > gamma * nphi0 )
      delta = min((z - l - w).^2 , (z - u - w).^2); 
      dk = min(delta(any([all([z-u<w, w<z-l], 2), w > z-l, w<z-u] ,2)));
      ek = min(0.5*gamma*nphi0, min(0.5*ek, dk)); 
    end%if
    step(it) = lambda;
    sit(it) = lit;
    nphik = norm(get_phi(l, u, z, w, ek, l_inf, u_inf, l_fin, u_fin)); 
    err(++it) = max(nphik, eps);
    epsilon(it) = ek;
    done = ( nphik < tol || it > maxit ) ; 
  end%while 

  
  idx = FREE * ones(size(z)); 
  idx(abs(z-l) < tol  ) =    LOWER; 
  idx(abs(u-z) < tol )  =    UPPER; 
  idx(  l==-Inf & u == Inf  ) = EQUALITY; 
  % 
  if (it == maxit )
    it = -it;
  end%if
  
  end%function


% evaluate the value of the function Phi which is the smoothed mid
% function.  
function phi = get_phi(l, u, z, w, mu, l_inf, u_inf, l_fin, u_fin)
  mu2 = mu*mu; 
  %phi = 2*z - (l+u) -sqrt((z-l-w).^2 + mu2) +sqrt((z-u-w).^2 + mu2); 
  phi = zeros(size(z));
  d = z(l_fin) - l(l_fin); 
  phi(l_fin) = phi(l_fin) + d - sqrt((d-w(l_fin)).^2  + mu2) ; 
  phi(l_inf) = phi(l_inf) + w(l_inf); 
  d = z(u_fin) - u(u_fin); 
  phi(u_fin) = phi(u_fin) + d + sqrt((d-w(u_fin)).^2  + mu2) ; 
  phi(u_inf) =  phi(u_inf) + w(u_inf); 
end%function

function [a, b] = get_ab(l, u, z, w, mu, l_inf, u_inf, l_fin, u_fin)
  mu2 = mu*mu;
  %b = (z-l-w)./sqrt( (z-l-w).^2 + mu2)  - (z-u-w)./sqrt( (z-u-w).^2 + mu2)  ; 
  b = zeros(size(l)); 
  b(l_inf) =  b(l_inf) - 1;
  b(u_inf) = b(u_inf) + 1;
  d = z(l_fin) - l(l_fin) - w(l_fin); 
  b(l_fin) = b(l_fin) + d./sqrt(d.^2 + mu2);
  d = z(u_fin) - u(u_fin) - w(u_fin); 
  b(u_fin) = b(u_fin) - d./sqrt(d.^2 + mu2);
  %b(find(b>1)) = 2 ; 
  %b(find(b<=1)) = 0 ; 
  a = 2-b; 
end%function
