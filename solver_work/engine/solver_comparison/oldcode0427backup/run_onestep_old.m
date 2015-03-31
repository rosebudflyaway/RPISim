function obj = run_onestep_old(obj, formulation, solver)
M = obj.M;
Gn = obj.Gn;
Gf = obj.Gf;
E  = obj.E;
U = obj.U;

NU = obj.NU;
nb = obj.nb;
nc = obj.nc;
nd = obj.nd;
h = obj.h;
FX = obj.FX;
PSI = obj.PSI;

% LCP  Lemke
if strcmp(formulation, 'LCP') && strcmp(solver, 'Lemke')
    MinvGn = M \ Gn;
    MinvGf = M \ Gf;
    
    A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
          Gf'*MinvGn   Gf'*MinvGf  E
          U            -E'         zeros(nc)];
    
    b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
          Gf'*(NU + M\FX*h);
           zeros(nc,1) ];
    
    % Solve with LEMKE
    [z, iter, err] = lemke( A, b, zeros(length(b)));
    
    % separate the normal err and  the frictional errs
    NU_ellp1 = NU + MinvGn * pn + MinvGf * pf + MinvPext;
    norm_err = z(1:nc, 1)' * (Gn' * NU_ellp1);
    tangent_vel = Gf' * NU_ellp1;
    for j = 1 : nc
        if  norm(tangent_vel( (j-1)*nd+1,j*nd)) > 1e-6  
        % sliding case
        slide_err = norm(U(j, j)*z(j, 1) - norm(z(nc+(j-1)*nd+1:nc+j*nd, 1)));
        else
        %sticking case
        stick_err = min(0, z((j-1)*nd+1:j*nd, 1));
        end
    end
    obj.z = z;
    obj.iter = iter;
    obj.norm_err = norm_err;
    obj.slide_err = slide_err;
    obj.stick_err = stick_err;
    
    % Handle potential errors
    if err ~= 0
        if size(err) == 1
            disp(['LCP Error: ' num2str(err)]);
        else
            disp('LCP Error');
        end
    end
end

% MCP PATH
if strcmp(formulation, 'mLCP') && strcmp(solver, 'PATH')
    
    A = [ -M             Gn                      Gf  zeros(6*nb,nc)  % Note: first line is negated
        Gn'            zeros(nc,nc*(2+nd))
        Gf'            zeros(nd*nc,(1+nd)*nc)  E
        zeros(nc,6*nb) U          -E'          zeros(nc)];
    b = [ M*NU + FX*h
        PSI / h
        zeros((nd+1)*nc,1) ];
    
    problem_size = length(b);
    z0 = zeros(problem_size, 1);
    big = 10^20;
    u = big*ones(problem_size, 1);
    l = [-big * ones(6*nb, 1);
        zeros((2+nd)*nc, 1)];
    [z, ~] = pathlcp(A, b, l, u, z0);
    NU_ellp1 = z(1:6*nb,1);
    norm_err = z(6*nb+1:6*nb+nc, 1)' * (Gn' * NU_ellp1);
    tangent_vel = Gf' * NU_ellp1;
    for j = 1 : nc
        if  norm(tangent_vel(6*nb+(j-1)*nd+1,6*nb+j*nd)) > 1e-6  
        % sliding case
        slide_err = norm(U(j, j)*z(6*nb+j, 1) - norm(Gf(:, (j-1)*nd+1:j*nd)*z(6*nb+nc+(j-1)*nd+1 : 6*nb+nc+j*nd, 1)));
        else
        %sticking case
        stick_err = min(0, z(6*nb+nc+(j-1)*nd+1:6*nb+nc+j*nd, 1));
        end
    end
    obj.z = z;
    obj.iter = [];
    obj.norm_err = norm_err;
    obj.slide_err = slide_err;
    obj.stick_err = stick_err;
    obj.z = z;
   % obj.err = z' * (A * z + b);
end

% MCP prox formulation  
if strcmp(formulation, 'mLCP') && strcmp(solver, 'fixed_point')
    obj.dynamics.M = M;
    obj.dynamics.Gn = Gn;
    obj.dynamics.Gf = Gf;
    obj.dynamics.h = h;
    obj.dynamics.FX = FX;
    obj.dynamics.PSI = PSI;
    obj.dynamics.NU = NU;
    obj.dynamics.U = U;
    obj.dynamics.E = E; 
    obj.h = h;
    obj.z = zeros(length(NU) + length(PSI) + nd *length(PSI) ,1);
    obj = mlcp_fixed_point(obj);
end

% mNCP prox formulation with PGS  AND with warm start
if strcmp(formulation, 'mNCP') && strcmp(solver, 'fixed_point_pgs_w')
    obj.dynamics.M = M;
    obj.dynamics.Gn = Gn;
    obj.dynamics.Gf = Gf;
    obj.dynamics.h = h;
    obj.dynamics.FX = FX;
    obj.dynamics.PSI = PSI;
    obj.dynamics.NU = NU;
    obj.dynamics.U = U;
    obj.h = h;
    obj.z = zeros(length(NU) + length(PSI) + 2*length(PSI) ,1);
    obj.warmstart = 'quadprogram';
    obj = mncp_fixed_point_pgs(obj);
    
end

% mNCP prox formulation with PGS  AND without warm start
if strcmp(formulation, 'mNCP') && strcmp(solver, 'fixed_point_pgs_wo')
    obj.dynamics.M = M;
    obj.dynamics.Gn = Gn;
    obj.dynamics.Gf = Gf;
    obj.dynamics.h = h;
    obj.dynamics.FX = FX;
    obj.dynamics.PSI = PSI;
    obj.dynamics.NU = NU;
    obj.dynamics.U = U;
    obj.h = h;
    obj.z = zeros(length(NU) + length(PSI) + 2*length(PSI) ,1);
    obj.warmstart = 'nowarm';
    obj = mncp_fixed_point_pgs(obj);
  
end


% mNCP prox formulation with blocked PGS  AND with warm start
if strcmp(formulation, 'mNCP') && strcmp(solver, 'fixed_point_blockpgs_w')
    obj.dynamics.M = M;
    obj.dynamics.Gn = Gn;
    obj.dynamics.Gf = Gf;
    obj.dynamics.h = h;
    obj.dynamics.FX = FX;
    obj.dynamics.PSI = PSI;
    obj.dynamics.NU = NU;
    obj.dynamics.U = U;
    obj.h = h;
    obj.z = zeros(length(NU) + length(PSI) + 2*length(PSI) ,1);
    obj.warmstart = 'quadprogram';
    obj = mncp_fixed_point(obj);
   
end

% mNCP prox formulation with blocked PGS  AND without warm start
if strcmp(formulation, 'mNCP') && strcmp(solver, 'fixed_point_blockpgs_wo')
    obj.dynamics.M = M;
    obj.dynamics.Gn = Gn;
    obj.dynamics.Gf = Gf;
    obj.dynamics.h = h;
    obj.dynamics.FX = FX;
    obj.dynamics.PSI = PSI;
    obj.dynamics.NU = NU;
    obj.dynamics.U = U;
    obj.h = h;
    obj.z = zeros(length(NU) + length(PSI) + 2*length(PSI) ,1);
    obj.warmstart = 'nowarm';
    obj = mncp_fixed_point(obj);
end

% LCP PGS
if strcmp(formulation, 'LCP') && strcmp(solver, 'PGS')
    MinvGn = M \ Gn;
    MinvGf = M \ Gf;
    
    A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
        Gf'*MinvGn   Gf'*MinvGf  E
        U            -E'         zeros(nc)];
    
    b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
        Gf'*(NU + M\FX*h);
        zeros(nc,1) ];
    x0 = zeros(length(b), 1);
    [x, err, iter, ~, ~, ~] = pgs(A, b, x0);
    obj.z = x;
    obj.err = err;
    obj.iter = iter;
end

% LCP Fischer - Newton 
if strcmp(formulation, 'LCP') && strcmp(solver, 'FischerNewton')
    MinvGn = M \ Gn;
    MinvGf = M \ Gf;
    
    A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
        Gf'*MinvGn   Gf'*MinvGf  E
        U            -E'         zeros(nc)];
    
    b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
        Gf'*(NU + M\FX*h);
        zeros(nc,1) ];
    x0 = zeros(length(b), 1);
    [x, err, iter, ~, ~, ~] = fischer_newton(A, b, x0);
    obj.z = x;
    obj.err = err;
    obj.iter = iter;
end

  
end

