function sim  = mncp_fixed_point_pgs(sim)
% The difference with mncp_fixed_point is that this loop changes to contact point
% wise. By converge on the first contact point, and then next, until Nth
M = sim.dynamics.M;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf;
h = sim.h;
FX = sim.dynamics.FX;
MinvPext = M \ FX*h;
PSI = sim.dynamics.PSI;
NU = sim.dynamics.NU;
U = sim.dynamics.U;
nc = length(PSI);
warm_start = sim.warmstart;
MinvGn     = M \ Gn;
MinvGf     = M \ Gf;
%r         = eig(Gn' * MinvGn)         % the parameter r
r          = 0.001;
rn         = r;
rf         = r;
maxIter1   = 100;      % the maximum number of iteration steps
maxIter2   = 20;
maxIter3   = 1000;
toler      = 1e-4;

% initial values for pf and s
pn = zeros(nc, 1);
pf = zeros(2*nc, 1);
norm_err = zeros(nc, maxIter2);
fric_err = zeros(nc, maxIter2);
err      = 1e6;
% to save the number of contacts: sliding; sticking; penetrate or detach.
stick_or_slide = zeros(nc, maxIter2);   % slide = 1
pene_or_deta   = zeros(nc, maxIter2);   % pene  = 1

 A       = Gn'*MinvGn;
for iter1 = 1:maxIter1
    % warm start the normal using Lemke
    b       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
   
    switch warm_start
        case 'Lemke'
            pn_ellp1 = lemke(A, b, pn);
      
        case 'quad_program'
            %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
            opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
            cons_A = [-1*eye(length(b));  -A];
            cons_b = [zeros(length(b), 1);  b];
            pn_ellp1 = quadprog(2*A, b, cons_A, cons_b, [], [], [], [], [], opts);
        otherwise
            pn_ellp1 = pn;
    end
    pn = pn_ellp1;
    % NU_ellp1 = update_vel(NU, MinvGn, pn, MinvGf, pf, MinvPext);
    NU_ellp1 = NU + MinvGn * pn + MinvGf * pf + MinvPext;
    err = 0;
    for   CT = 1:nc  % iter2 iterations over every contact
        for iter2 = 1: maxIter2
            [pn_ellp1(CT, 1), flag, err1]  = update_normal(pn(CT, 1),  rn, PSI(CT, 1), h, Gn(:, CT), NU_ellp1);
            if err1 ~= 0 
                err1 = err1 / abs(err1);
            else
                err1 = 0;
            end
            pene_or_deta(CT, iter2)  = flag;
            norm_err(CT, iter2) = err1;
            if err1 < toler 
                pn(CT, 1) = pn_ellp1(CT, 1);
                break;
            end
            pn(CT, 1) = pn_ellp1(CT, 1);
        end
        
        for iter3 = 1:maxIter3
            [pf_ellp1(2*CT-1:2*CT, 1), flag, err2]  = update_fric(pf(2*CT-1:2*CT, 1), rf, Gf(:, 2*CT-1: 2*CT), NU_ellp1, pn_ellp1(CT, 1), U(CT, CT));
            if err2 ~= 0 
                err2 = err2 / abs(err2);
            else
                err2 = 0;
            end
            stick_or_slide(CT, iter3) = flag;
            fric_err(CT, iter3)   = err2;
            if(err2 < toler)
                pf(2*CT-1:2*CT, 1) = pf_ellp1(2*CT-1:2*CT, 1);
                break;
            end
            pf(2*CT-1:2*CT, 1) = pf_ellp1(2*CT-1:2*CT, 1);
        end
    end 
%     if iter1 == maxIter1
%         fprintf('The maximum iteration step has been reached \n');
%     end
    err3 = 0; % err3 is the total error for all the contacts points 
    for CT = 1 : nc    
        err3 = err3 + norm_err(CT, length(norm_err(CT, :))) + fric_err(CT, length(fric_err(CT,:))); 
    end
    if err3 < toler
        break;
    end
end
NU_ellp1 = NU + MinvGn * pn + MinvGf * pf + MinvPext;

z  =[NU_ellp1; pn; pf];
stick_num = length(stick_or_slide(:, iter2) < 0);
slide_num = length(stick_or_slide(:, iter2) > 0);
num  = [nc, stick_num, slide_num, err, iter2];
sim.z = z;
sim.solution.iter = iter1 + iter2 + iter3;
sim.solution.err = err3;
sim.solution.num = num;
end


function [NU_ellp1] = update_vel(NU, MinvGn, pn, MinvGf, pf, MinvPext)
NU_ellp1 = NU + MinvGn * pn + MinvGf * pf + MinvPext;
end

function [pn_ellp1, flag, err] = update_normal(pn, rn, PSI, h, Gn, NU_ellp1)
% scalar operations below
% Pn_ellp1     = prox(Pn - rn (PSI/h + Gn'*NU_ellp1))
rhon = PSI/h + Gn'* NU_ellp1;
pn_temp = pn - rn*rhon;
% The normal force can not be negative, project onto the non negative space
pn_ellp1 = pn_temp;
if pn_ellp1 <= 0   % detach
    flag = -1;
    pn_ellp1 = 0;
else
    flag = 1;      % >0  contact
end
err = PSI * pn_ellp1;

end

function [pf_ellp1, flag, err] = update_fric(pf, rf, Gf, NU_ellp1, pn_ellp1,U)
% Pf_ellp1 = prox(Pf - rf * (Gf * NU_ellp1 + s))
% scalar operations below 
rhof = Gf' * NU_ellp1;
pf_temp = pf - rf * rhof;
rel_vel = Gf' * NU_ellp1;
rel_vel_dir = rel_vel / norm(rel_vel);
pf_ellp1 = zeros(2, 1);
% The frictional force should be projected inside or onto the cone
pf_mag = norm(pf_temp);
pf_dir = pf_temp / pf_mag;

% sliding case , error is frictional force - mu * normal force
    if ( pf_mag >= U * pn_ellp1)
        flag = 1;
        pf_mag = U*pn_ellp1;
        pf_ellp1 = pf_mag * pf_dir;
        err = (1/pi)*(acos(rel_vel_dir' * pf_dir) - pi);
    else 
        % sticking case; error is the norm of relative velocity
        flag = -1;
        err = norm(rel_vel);
    end
end
