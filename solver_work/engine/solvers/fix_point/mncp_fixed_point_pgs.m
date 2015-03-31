function solution  = mncp_fixed_point_pgs(sim)
% The difference with mncp_fixed_point is that this loop changes to contact point
% wise. By converge on the first contact point, and then next, until Nth
Minv = sim.Ndynamics.Minv;
Gn = sim.Ndynamics.Gn;
Gf = sim.Ndynamics.Gf;
h = sim.Ndynamics.h;
U = sim.Ndynamics.U;
FX = sim.Ndynamics.Forces;
MinvPext = Minv * FX * h;
PSI = sim.contacts.PSI;
NU = sim.Ndynamics.Vel;
E  = sim.Ndynamics.E;

nc = length(PSI);
nd = 2;
A = zeros(4*nc, 4*nc);
A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
      Gf'*MinvGn   Gf'*MinvGf  E
      U            -E'         zeros(nc)];

if exist('warm_start')
    warm_start = sim.warmstart;
else
    warm_start = 'quad_program';
end
if isfield(sim, 'constraints')
    % bilateral constraints
    Gb = sim.Ndynamics.Gb;
    Bounds = sim.constraints.bounds';
    Violation = sim.constraints.violation;
    nj = length(Violation);
    MinvGb     = Minv * Gb;
    [~, pb_size] = size(Gb);
    pb = zeros(pb_size, 1);
    rb = 0.3 / eigs(Gb' * MinvGb, 1);
end
% bilateral constraints
MinvGn     = Minv * Gn;
MinvGf     = Minv * Gf;

ErrorMetric = sim.solver.errmetric;
TuneParams = sim.solver.TuneParams;
max_iter = TuneParams.max_iter;
tol = TuneParams.tol;
alpha = TuneParams.alpha;

%converge = zeros(maxIter1, 1);
% initial values for pf
% pn = zeros(nc, 1);
% pf = zeros(2*nc, 1);
% s = zeros(nc, 1);

pn = zeros(nc, 1);
pf = zeros(2*nc, 1);
s = zeros(nc, 1);
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

z = [pn ; pf ; s];
b = zeros(length(z));
b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc); 

% norm_err = zeros(nc, maxIter1);
% fric_err = zeros(nc, maxIter1);
% to save the number of contacts: sliding; sticking; penetrate or detach.
% stick_or_slide = zeros(nc, maxIter1);   % slide = 1
% pene_or_deta   = zeros(nc, maxIter1);   % pene  = 1

% Anorm       = Gn'*MinvGn;
% bnorm       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
% switch warm_start
%     case 'Lemke'
%         pn_ellp1 = lemke(Anorm, bnorm, pn);
%     case 'quad_program'
%         %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
%         opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
%         cons_A = [-1*eye(length(bnorm));  -Anorm];
%         cons_b = [zeros(length(bnorm), 1);  bnorm];
%         pn_ellp1 = quadprog(2*Anorm, bnorm, cons_A, cons_b, [], [], [], [], [], opts);
%     otherwise
%         pn_ellp1 = pn;  % when there is no warm start
% end
% pn = pn_ellp1;

err = Inf;
for iter1 = 1 : max_iter
    new_NU = NU + MinvGn*pn + MinvGf * pf + MinvPext;
    % normal part
    for CT = 1 : nc
        old_pn = pn(CT, 1);
        rho_n = Gn(:, CT)' * new_NU + PSI(CT, 1) / h;
        new_pn = project(old_pn - rn * (rho_n + alpha * old_pn));
        pn(CT, 1) = new_pn;
    end
    % frictional part
    for CT = 1 : nc
        old_f1 = pf(2*CT-1, 1);
        rho_f1 = Gf(:, 2*CT-1)' * new_NU + E(2*CT-1, :) * s;
        old_f2 = pf(2*CT, 1);
        rho_f2 = Gf(:, 2*CT)' * new_NU + E(2*CT, :) * s;
        new_f1 = old_f1 - rf * rho_f1;
        new_f2 = old_f2 - rf * rho_f2;
        new_f = [new_f1, new_f2];
        if norm(new_f) > U(CT, CT) * norm(pn(CT, 1))
            new_f1 = new_f1 / norm(new_f);
            new_f2 = new_f2 / norm(new_f);
        end
        pf(2*CT-1:2*CT, 1) = [new_f1; new_f2];
    end
    
    for i = 1 : nc
        old_s = s(i, 1);
        rho_s = U(i, i) * pn - E(:, i)' * norm(pf(2*i-1:2*i, 1));
        new_s = project(old_s - rs * (rho_s  + alpha * old_s));
        s(i, 1) = new_s;
    end
    z = [pn, pf, s];
    solution = updateSolutionData(solution, ErrorMetric, iter1, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU);
    old_err = err;
    err = solution.total_error(iter);
%    if err < tol && abs(err - old_err) / abs(old_err) < tol
    if err < tol   
        break;
    end
end

end

 

function pn_ellp1 = update_normal(pn, rn, PSI, h, Gn, NU_ellp1)
% scalar operations below
% Pn_ellp1     = prox(Pn - rn (PSI/h + Gn'*NU_ellp1))
rhon = PSI/h + Gn'* NU_ellp1;
pn_temp = pn - rn*rhon;
% The normal force can not be negative, project onto the non negative space
pn_ellp1 = pn_temp;
% if pn_ellp1 <= 0   % detach
%     flag = -1;
%     pn_ellp1 = 0;
% else
%     flag = 1;      % >0  contact
% end
% err = PSI * pn_ellp1;
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