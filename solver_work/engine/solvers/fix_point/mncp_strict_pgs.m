function solution  = mncp_strict_pgs(sim)
% The difference with mncp_fixed_point is that this loop changes to contact point
% wise. By converge on the first contact point, and then next, until Nth
Minv = sim.Ndynamics.Minv;
Gn = sim.Ndynamics.Gn;
Gf = sim.Ndynamics.Gf;
h = sim.Ndynamics.h;
U = sim.Ndynamics.U;
FX = sim.Ndynamics.Forces;
NU = sim.Ndynamics.Vel;
PSI = sim.contacts.PSI;
nc = length(PSI);
nd = sim.Ndynamics.nd;

MinvGn     = Minv * Gn;
MinvGf     = Minv * Gf;
MinvPext = Minv * FX * h;


ErrorMetric = sim.solver.errmetric;
ErrorMetric.metricName = 'ChenChen_Kanzow';

TuneParams = sim.solver.TuneParams;
max_iter = TuneParams.max_iter;  % The maximum number of iteration steps
tol = TuneParams.tol;
r   = TuneParams.r;
rn = r;  rf = r;  rs = r;

 
% should be less than 2/eig
%  rn = factor / eigs(Gn' * MinvGn, 1);
%  rf = 3 * factor / eigs(Gf' * MinvGf, 1);
%  temp = 1/2 * (eigs(Gn' * MinvGn, 1) + eigs(Gf' * MinvGf, 1));
%  rs = 3 * factor / temp;

err = Inf;
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc);
pn = zeros(nc, 1);
pn_ellp1 = zeros(nc, 1);
pf = zeros(2*nc, 1);
pf_ellp1 = zeros(2*nc, 1); % preallocate for the following row by row update
s = zeros(nc, 1);
s_ellp1 = zeros(nc, 1); % preallocate for the following row by row update
warm_start = '';

z = [pn; pf; s];
A = zeros(length(z), length(z));

solution  = struct();
solution.total_error = zeros(max_iter, 1);
solution.normal_error = zeros(max_iter, 1); 
solution.friction_error = zeros(max_iter, 1) ;
solution.stick_or_slide = zeros(max_iter, 1);
solution.z = zeros(4*nc, max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);
% Warm start the normal impulse
switch warm_start
    case 'Lemke'
        Anorm       = Gn'*MinvGn;
        bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        pn  = lemke(Anorm, bnorm, pn);
    case 'quad_program'
        Anorm       = Gn'*MinvGn;
        bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
        opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
        cons_A = [-1*eye(length(bnorm));  -Anorm];
        cons_b = [zeros(length(bnorm), 1);  bnorm];
        pn  = quadprog(2*Anorm, bnorm, cons_A, cons_b, [], [], [], [], [], opts);
    otherwise
        %disp('No warm start metric is used');
end
 
 
for iter = 1 : max_iter  % one iteration is a loop over all the contact  
    NU_ellp1 = NU + MinvGn*pn + MinvGf*pf + Minv*FX*h;  % update NU_ellp1, but the base is always NU;
    solution  = updateSolutionData(solution, ErrorMetric, iter, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU_ellp1);
    old_err = err;
    err     = solution.total_error(iter);
    if err < tol && abs(old_err - err) / abs(old_err) < tol
        break;
    end
    
    for CT = 1 : nc       % loop over all the contacts, each time solve one contact: both normal and friction
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pn_ellp1(CT, 1)  = update_normal(pn(CT, 1),  rn, PSI(CT, 1), h, Gn(:, CT), NU);
        
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % friction part
        pf_ellp1(2*CT-1:2*CT, 1)  = update_fric(pf(2*CT-1:2*CT, 1), rf, Gf(:, 2*CT-1: 2*CT), NU, pn_ellp1(CT, 1), U(CT, CT));
 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [s_ellp1(CT, 1)] = update_slide(s(CT,1), pn(CT, 1), pf(2*CT-1:2*CT, 1), U(CT, CT), rs);
        
      
    end
    z = [pn_ellp1; pf_ellp1; s_ellp1];
    pf = pf_ellp1;
    pn = pn_ellp1;
    s = s_ellp1;
end
end

function pn_ellp1 = update_normal(pn, rn, PSI, h, Gn, NU_ellp1)
% scalar operations below
% Pn_ellp1     = prox(Pn - rn (PSI/h + Gn'*NU_ellp1))
rhon = PSI/h + Gn'* NU_ellp1;
pn_temp = pn - rn*rhon;
% The normal force can not be negative, project onto the non negative space
pn_ellp1 = pn_temp;
 
end

function pf_ellp1 = update_fric(pf, rf, Gf, NU_ellp1, pn_ellp1,U)
% Pf_ellp1 = prox(Pf - rf * (Gf * NU_ellp1 + s))
rhof = Gf' * NU_ellp1;
pf_temp = pf - rf * rhof;
%rel_vel = Gf' * NU_ellp1;
%rel_vel_dir = rel_vel / norm(rel_vel);
%pf_ellp1 = zeros(2, 1);
% The frictional force should be projected inside or onto the cone
% sliding case , error is frictional force - mu * normal force
if pn_ellp1 > 1e-6  && (norm(pf_temp) > U * pn_ellp1)
        pf_temp = U*pn_ellp1 * (pf_temp / (norm(pf_temp)));    
end
 
pf_ellp1 = pf_temp;
end

function [s_ellp1] = update_slide(s, pn, pf, U, rs)
     rhos = U*pn - norm(pf);
     sTemp = s - rs * rhos;
     if sTemp < 0 
         sTemp = 0;
     end
     s_ellp1 = sTemp;
end

function b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc)
b = [ Gn'*(NU + MinvPext) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
      Gf'*(NU + MinvPext);
      zeros(nc,1) ];    
end
 


    % right now in the code, we have only spherical joint, which the size
    % should be 3
    % we don't loop joint by joint, since provided the big jacobian matrix
    % as a whole, here will solve all the Pb at the same time. Besides, the
    % Pb to solve is an equation, no iteration is needed.
    
    % Idety = ones(pb_size, 1);
    % how does this come from? we have pb in the form of equations,
    % rather than complementarity form, how does the prox come then ????
    % The prox here actually is we didn't project onto the positive plane,
    % but the complementarity condition doesn't exist in the physical
    % model, don't do projection onto the positive plane here. 
    % pb = min(Bounds(:, 2) * Idety, max(Bounds(:, 1) * Idety, pb - rb * (Gb' * NU_ellp1)));    