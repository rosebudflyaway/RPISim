function solution = mncp_fixed_point(sim)
% This is solved in the following order: 
% 1 ------------------------pn
% 2 ------------------------pf

Minv = sim.Ndynamics.Minv;
Gn = sim.Ndynamics.Gn;
Gf = sim.Ndynamics.Gf; 
h = sim.Ndynamics.h;
FX = sim.Ndynamics.Forces;

PSI = sim.Ndynamics.PSI;
NU = sim.Ndynamics.Vel;
U = sim.Ndynamics.U;
nc = length(PSI);
nd = sim.Ndynamics.nd;

 
MinvGn     = Minv * Gn;
MinvGf     = Minv * Gf;
MinvPext = Minv* FX*h;

ErrorMetric = sim.solver.errmetric;
ErrorMetric.metricName = 'ChenChen_Kanzow';

TuneParams = sim.solver.TuneParams;
max_iter   = TuneParams.max_iter;
tol        = TuneParams.tol;
r          = TuneParams.r;

rn = r;  rf = r; 

% initial values for pf and s
pn = zeros(nc, 1);
pf = zeros(nc*nd, 1);
s  = zeros(nc, 1);
warm_start = '';  % no warm start to tune the parameters
 
z = [pn; pf ];
A = zeros(length(z), length(z));
solution  = struct();  
solution.normFBerror = zeros(max_iter, 1);
solution.fricFBerror = zeros(max_iter, 1);
solution.total_error = zeros(max_iter , 1);
solution.normal_error = zeros(max_iter , 1); 
solution.friction_error = zeros(max_iter , 1);
solution.stick_or_slide = zeros(max_iter, 1);
solution.z = zeros(length(z), max_iter);
solution.iterations = 0;
solution.direction_error = zeros(max_iter, 1);
solution.copositive_normal_error = zeros(max_iter, 1);
solution.copositive_friction_error = zeros(max_iter, 1);
solution.normal_neg_error = zeros(max_iter, 1);
solution.fric_neg_error = zeros(max_iter, 1);


b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc);
err = Inf;

%warm start the normal using Lemke
switch warm_start
    case 'Lemke'
        Anorm   = Gn'*MinvGn;
        bnorm   = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        pn      = lemke(Anorm, bnorm, zeros(length(bnorm), 1));
    case 'quad_program'
        Anorm   = Gn'*MinvGn;
        bnorm   = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
        %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
        opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
        cons_A = [-1*eye(length(bnorm));  -Anorm];
        cons_b = [zeros(length(bnorm), 1);  bnorm];
        pn     = quadprog(2*Anorm, bnorm, cons_A, cons_b, [], [], [], [], [], opts);
    otherwise
        %disp('no warm start');
end

% make sure the NU
%%%%%%%%%%%%%%%%%%%%%%%%%%% START THE ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%
for iter = 1:max_iter 
    NU_ellp1 = NU + MinvGn*pn + MinvGf*pf + Minv*FX*h;  % update NU_ellp1, but the base is always NU;
    solution        = updateSolutionData(solution, ErrorMetric, iter, A, z, b, s, U, pn, PSI, pf, nd, Gf, NU_ellp1);
    old_err         = err;
    err             = solution.total_error(iter);
    if err < tol  && abs(err - old_err) / abs(old_err) <  tol
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pn_ellp1        = update_normal(pn, rn, PSI, h, Gn, NU);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pf_ellp1        = update_fric(pf, rf, Gf, NU, pn_ellp1, nc, U);
    z = [pn_ellp1; pf_ellp1];
    % err = norm(z' * (A * z + b));
    
    pn = pn_ellp1;
    pf = pf_ellp1;
end
 
end

function [pn_temp, pn_ellp1]   = update_normal(pn, rn, PSI, h, Gn, NU_ellp1)
% Pn_ellp1            = prox(Pn - rn (PSI/h + Gn'*NU_ellp1))
rhon                  = PSI/h + Gn'* NU_ellp1;
pn_temp               = pn - rn*rhon;
% The normal force can not be negative, project onto the non negative space
pn_ellp1              = pn_temp;
pn_ellp1(pn_ellp1<0)  = 0;
end

function [pf_temp, pf_ellp1]   = update_fric(pf, rf, Gf, NU_ellp1, pn_ellp1, nc, U)
% Pf_ellp1            = prox(Pf - rf * (Gf * NU_ellp1 + s))
rhof                  = Gf' * NU_ellp1;
pf_temp               = pf - rf * rhof;
rel_vel_dir           = Gf' * NU_ellp1;
pf_ellp1              = zeros(nc*2, 1);
% The frictional force should be projected inside or onto the cone
for   i = 1:nc
    pf_mag  = norm(pf_temp(2*i-1: 2*i, 1));
    if ( pf_mag > U(i, i) * pn_ellp1(i))
        pf_dir  = rel_vel_dir(2*i-1: 2*i, 1);
        pf_dir  = -pf_dir / norm(pf_dir);
        pf_ellp1(2*i-1:2*i, 1) = pf_mag * pf_dir;
    end
end
end

function b = setb(Gn, Gf, NU, MinvPext, PSI, h, nc)
b = [ Gn'*(NU + MinvPext) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
      Gf'*(NU + MinvPext);
      zeros(nc,1) ];    
end


