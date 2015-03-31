% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%


function [z, w, idx, it, err, errv, epsilon, step, sit] = zhang_gao_smoothing (M, q, l, u, maxit, tol, z0)

  % usage:  [z, w, idx, it, err] = zhang_gao_smoothing (M, q, l, u, z0)
  %
  % Implement globally convergent smoothing Newton method of Liping
  % Zhang and Ziyou Gao from 'Quadratic one-step smoothing new method
  % of P_0 LCP without strict complementarity.  Applied Mathematics and
  % Computation, 140 (2003) pp 367-379.


  % Index definition
  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;
 
  l_fin = find(isfinite(l));
  u_fin = find(isfinite(u));
  u_inf = find(isinf(u));
  l_inf = find(isinf(l));

  errv = [];
  if ( ~ exist('tol', 'var') )
    tol = 1E-10;
  end%if

  n = length(q); 
  
  if ( ~ exist('z0', 'var') ) 
    z0 = zeros(size(q));
  end%if
  if ( ~ exist('maxit', 'var') )
    maxit = 50;
  end%if
  z = z0; 
  w = M*z + q; 

  % fix first set of parameters 
  phi_k = get_phi(l, u, z, w, 0, l_inf, u_inf, l_fin, u_fin);
  theta_0 = norm( phi_k ) ; 
  mu0 =  0.1*theta_0;
  mu_k    = mu0; 
  mu_bar = mu0; 

  % get initial norm
  phi_k = get_phi(l, u, z, w, mu_k, l_inf, u_inf, l_fin, u_fin);
  theta_k = norm(  [ mu_k ; phi_k ] ) ; 
  if ( theta_k > tol && mu_bar > 0 )
    gamma = min(1E-2, min(0.5/theta_k, 0.5/mu_bar));
  else
    gamma = 1E-2;
  end %if
  
  
  % 
  % Fix some more parameters.  Both of these affect the algorithms in
  % complicated ways. 
  
  delta = 0.4;			% line search attenuation
  sigma = 1E-8;			% Amijo line search
  eta   = gamma*mu_bar; 	% another Armijo parameter
  alpha = sigma*(1-eta) ; 

  done = 0 ; 
  it = 1; 
  if ( theta_0 < tol ) 
    done = 1; 
    errv = [errv; max(eps, theta_0)];
  else
    done = theta_k < tol; 
    errv = [errv; max(eps, theta_k)];
  end%if

  epsilon(it) = mu_k;

  while ( ~ done ) 

    rho_k = gamma*theta_k*min(1, theta_k); 
    
    idx = FREE * ones(size(z)); 
    idx(abs(z-l) < tol  ) =    LOWER; 
    idx(abs(u-z) < tol )  =    UPPER;  
    [du, dz, dw ] = ...
	get_search_direction(M, z, w, l, u, ...
			     mu_k, rho_k, phi_k, l_inf, u_inf, l_fin, ...
			     u_fin, idx);

    % perform the line search
    delta_i = 1/delta;
    lit = 0; 
    while (1) 
       delta_i = delta_i * delta ; 
       z_i  = z + delta_i*dz; 
       w_i  = w + delta_i*dw; 
       mu_i = mu_k + delta_i*du; 
       phi_i = get_phi(l, u, z_i, w_i, mu_i, l_inf, u_inf, l_fin, u_fin);
       theta_i = norm(  [ mu_i ;  phi_i ]  ) ; 
       
       if(theta_i < ( 1 - alpha*delta_i)*theta_k)
           break;
       end
       
       lit = lit + 1; 
    end
    step(it) = delta_i;
    sit(it) = lit;
    z = z + delta_i*dz; 
    w = w + delta_i*dw; 
    mu_k= mu_k + delta_i*du; 
    epsilon(it) = mu_k ; 
    
    phi_k = get_phi(l, u, z, w, mu_k, l_inf, u_inf, l_fin, u_fin);
    theta_k = norm(  [ mu_k ;  phi_k ] ) ; 
    errv = [errv;theta_k];
    it = it + 1;
		   
    done = (theta_k < tol || it > maxit); 
  end%while
  
  idx = FREE * ones(size(z)); 
  idx(abs(z-l) < tol  ) =    LOWER; 
  idx(abs(u-z) < tol )  =    UPPER; 
  idx(  l==-Inf & u == Inf  ) = EQUALITY; 

  if (it == maxit )
    it = -it;
  end%if

  
  %finish off with a direct solve without perturbation.
  z = solve_subproblem(M, q, l, u, idx);
  w = M*z + q; 
  err = errv(end);  
end%function



% get the search direction
function [du, dz, dw ] = get_search_direction(M, z, w, l, u, mu, rho, phi, l_inf, u_inf, l_fin, u_fin, idx)
    % get derivative factors
    [d1, d2, d3] = get_dphi(l, u, z, w, mu, l_inf, u_inf, l_fin, u_fin);
    % get value of du first
    du = mu*(-1 + rho); 
    if ( 1 )
      % get dz value
      dz = (sparse(diag(d3)) + diag(d2)*M)\(-phi - du*d1); 
      % get dw value
      dw = M*dz;
    else
      % use approximate to solve
      dz = solve_subproblem(M, (phi + du * d1), l, u, idx);
      dw = M*dz;
    end %if    finite
end %function

				     

% evaluate the value of the function Phi which is the smoothed mid
% function.  
function phi = get_phi(l, u, z, w, mu, l_inf, u_inf, l_fin, u_fin)
  mu2 = mu*mu; 

  phi = zeros(size(z));
  d = z(l_fin) - l(l_fin); 
  phi(l_fin) = phi(l_fin) + d - sqrt((d-w(l_fin)).^2  + mu2) ; 
  phi(l_inf) =phi(l_inf) + w(l_inf); 
  d = z(u_fin) - u(u_fin); 
  phi(u_fin) = phi(u_fin) + d + sqrt((d-w(u_fin)).^2  + mu2) ; 
  phi(u_inf) = phi(u_inf) + w(u_inf); 

end%function



% 
%  Here, evaluate derivatives of the smoothing function.
%  The derivative info is returned in 3 vectors: d1 is the partial
%  derivative with respect to the smoothing factor, mu, d2 is the
%  gradient in the direction of 'b' for the 'smoothed mid function'
%  phi(mu, a, b, c) and d2  = 2 - d1; 
%
function [d1, d2, d3] = get_dphi(l, u, z, w, mu, l_inf, u_inf, l_fin, u_fin)
  mu2 = mu*mu;
  d1 = zeros(size(l)); 
  d2 = zeros(size(l)); 
  % work on infinite lower bounds: d1(l_inf) = 0 
  d2(l_inf) = d2(l_inf) - 1; 

  % work on finite lower bounds
  d = z(l_fin) - l(l_fin) - w(l_fin); 
  f = 1./sqrt(d.^2 + mu2); 

  d1(l_fin) = d1(l_fin) - mu*f; 
  d2(l_fin) = d2(l_fin) + d.*f; 
  % work on infinite upper bounds: d1(l_inf) = 0 
  d2(u_inf) = d2(u_inf) + 1;
  % work on finite upper bounds
  d = z(u_fin) - u(u_fin) - w(u_fin); 
  f = 1./sqrt(d.^2 + mu2); 

  d1(u_fin) = d1(u_fin) + mu*f; 
  d2(u_fin) = d2(u_fin) - d.*f; 

  d3 = 2-d2; 
end%function
