function sim = mncp_fixed_point(sim)
% The difference with version_1 is that this loop changes to contact point
% wise. 

M = sim.dynamics.M;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf; 
h = sim.h;
FX = sim.dynamics.FX;
MinvPext = M \ FX*h;
PSI = sim.dynamics.PSI;
NU = sim.dynamics.NU;
nc = length(PSI);
U = sim.dynamics.U;
warm_start = sim.warmstart;
MinvGn     = M \ Gn;
MinvGf     = M \ Gf;
 
nf         = size(Gf, 2);              % number of contacts * friction_directions(2)

%r         = eig(Gn' * MinvGn)         % the parameter r
r          = 1e-2;
rn         = r;
rf         = r;
maxIter1   = 10;      % the maximum number of iteration steps
maxIter2   = 1000;
toler      = 1e-4;
% initial values for pf and s
pn = zeros(nc, 1);
pf = zeros(nf, 1);

% norm_err = zeros(maxIter2, 1);
% stick_err = zeros(maxIter2, 1);
% slide_err = zeros(maxIter2, 1);
%stick_or_slide = zeros(num_contacts, 1);
%pene_or_deta   = zeros(num_contacts, 1);

for iter1 = 1:maxIter1
    % warm start the normal using Lemke
    A       = Gn'*MinvGn;
    b       = Gn'*(NU + MinvPext) + PSI/h - Gn'*MinvGf*pf;
    switch warm_start
        case 'Lemke'
            pn      = lemke(A, b, zeros(length(b), 1));
        case 'quad_program'
            %opts  = optimset('Algorithm', 'active-set', 'Display', 'off');
            opts  = optimset('Algorithm', 'interior-point-convex', 'Display', 'off');
            cons_A = [-1*eye(length(b));  -A];
            cons_b = [zeros(length(b), 1);  b];
            pn     = quadprog(2*A, b, cons_A, cons_b, [], [], [], [], [], opts);
    end
    
    for iter2 = 1: maxIter2
        NU_ellp1        = update_vel(NU, MinvGn, pn, MinvGf, pf, MinvPext);
        pn_ellp1        = update_normal(pn, rn, PSI, h, Gn, NU_ellp1);
        pf_ellp1        = update_fric(pf, rf, Gf, NU_ellp1, pn_ellp1, nc, U);
        norm_err         = PSI' * pn_ellp1;
        % ##################################################################
        % ######## The frictional error might be important to check#########
        % (1). The magnitude is inside the cone
        % (2). The direction is colinear with the sliding speed
        % check all the contacts
        rel_vel         = Gf' * NU_ellp1;
        slide_num = 0;
        stick_num = 0;
        for   i = 1:nc
            stick_err = 0;
            slide_err = 0;
            if norm(rel_vel(2*i-1: 2*i, 1)) < eps
                % sticking case
                stick_num   =  stick_num  +  1;
                stick_err = norm(pf_ellp1(2*i-1:2*i, 1)) - U(i, i)  * pn_ellp1(i, 1);
                
                 
            else
                % sliding case
                slide_num  =  slide_num + 1;
                if abs(norm(pf_ellp1(2*i-1:2*i, 1)) - U(i, i) * pn_ellp1(i, 1)) > eps
                    slide_err   =  norm(pf_ellp1(2*i-1:2*i, 1)) - U(i, i) * pn_ellp1(i, 1);
                end
            end
        end
        
        err   =  norm_err + stick_err  + slide_err;
        if(err < toler)
            pn    = pn_ellp1;
            pf    = pf_ellp1;
            NU    = NU_ellp1;
            z  =[pn; pf; NU];
            err = norm_err + stick_err + slide_err;
            num  = [stick_num; slide_num; iter1];
            break;
        end
        pn    = pn_ellp1;
        pf    = pf_ellp1;
        NU    = NU_ellp1;
        %pn(pn>1e4) = zeros(length(find(pn>1e4)),1);
        z  =[NU; pn; pf; NU];
        err = norm_err + stick_err + slide_err;
        num  = [stick_num; slide_num; iter1];
    end
    sim.z = z;
    sim.solution.err = err;
    sim.solution.iter = iter2;
    sim.solution.num = num;
    %  end
end
 
end



function [NU_ellp1]   = update_vel(NU, MinvGn, pn, MinvGf, pf, MinvPext)
NU_ellp1              = NU + MinvGn * pn + MinvGf * pf + MinvPext;
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

