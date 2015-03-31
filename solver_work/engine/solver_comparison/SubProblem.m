% This is the subproblem formulation to solve one simulation step of the
% whole simulations. Here, the matrices are formulated and the
% corresponding solvers are called.
% Input: 
% obj ------------> obj.cur_fm  
% formulation ----> 'LCP', 'NCP'
% solver----------> 'Lemke', 'PATH'
function solution = SubProblem(obj, formulation, solver)
M = obj.dynamics.M;
Gn = obj.dynamics.Gn;
Gf = obj.dynamics.Gf;
E  = obj.dynamics.E;
U = obj.dynamics.U;
nd = obj.dynamics.nd;
NU = obj.bodies.velocities';
nb = length(obj.bodies.masses);
nc = length(obj.contacts.depth);
nbilat = 0;
if isfield(obj, 'constraints')
    % bilateral constraints
    nbilat = length(obj.constraints.pairs);
end
h = obj.dynamics.h;
FX = obj.bodies.forces';
% the depth is actually the penetration here 
PSI = -obj.contacts.depth';
% for the use of prox solvers
solution.z = zeros(length(NU) + length(PSI) + nd*length(PSI) ,1);
MinvGn = M \ Gn;
MinvGf = M \ Gf;
if strcmp(formulation, 'LCP')
    A_LCP = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc);
              Gf'*MinvGn   Gf'*MinvGf  E;
              U            -E'         zeros(nc)];
    
    b_LCP = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
                  Gf'*(NU + M\FX*h);
                    zeros(nc,1) ];
elseif strcmp(formulation, 'mLCP')
    A_MCP = [ -M,                  Gn ,                     Gf,   zeros(6*nb,nc);  % Note: first line is negated
              Gn'            zeros(nc,nc*(2+nd));
              Gf'            zeros(nd*nc,(1+nd)*nc)       E;
             zeros(nc,6*nb)       U          -E'          zeros(nc)];
    
    b_MCP = [ M*NU + FX*h
                PSI / h
            zeros((nd+1)*nc,1) ];
end

% Lemke
if  strcmp(formulation, 'LCP') && strcmp(solver, 'Lemke')
    A = A_LCP;
    b = b_LCP;
    [z, iter, err] = lemke( A, b, zeros(length(b)));
    solution.z = z;
    solution.iter = iter;
    % TODO: put the detailed error here: Norm, Frictional
    solution.err = z' * (A*z + b);    
    % Handle potential errors
    if err ~= 0
        if size(err) == 1
            disp(['LCP Error: ' num2str(err)]);
        else
            disp('LCP Error');
        end
    end


% PATH
elseif  strcmp(formulation, 'mLCP') && strcmp(solver, 'PATH')
    A = A_MCP;
    b = b_MCP;
    problem_size = length(b);
    z0 = zeros(problem_size, 1);
    big = 10^20;
    u = big*ones(problem_size, 1);
    l = [-big * ones(6*nb, 1);
          zeros((2+nd)*nc, 1)];
    [z, ~] = pathlcp(A, b, l, u, z0);
    solution.z = z;
    solution.iter = [];
    solution.err = z' * (A * z + b);
    % the detailed error information
    NU = z(1:6*nb, 1);
    pn = z(6*nb+1 : 6*nb+nc, 1);  
    pf = z(6*nb+nc+1 : 6*nb+nc+nc*nd, 1);
    rel_vel_n = Gn' * NU;     % nc X 1
    rel_vel_t = Gf' * NU;     % (nc*nd) X 1   
    % normal error
    norm_err = pn' * rel_vel_n;
    % resultant frictional force and velocity on the tangential plane
    resultant_f  = zeros(nc, 2);
    resultant_v  = zeros(nc, 2);
    theta = 2*pi/nd;
    % get the friction and velocity in the tangential plane as resultant force and velocity. 
    for i = 1 : nc
        for  j = 1 : nd
            resultant_f(i, 1) = resultant_f(i, 1) + pf((i-1)*nd+j, 1) * cos((j-1)*theta);
            resultant_f(i, 2) = resultant_f(i, 2) + pf((i-1)*nd+j, 1) * sin((j-1)*theta);
            resultant_v(i, 1) = resultant_v(i, 1) + rel_vel_t((i-1)*nd+j, 1) * cos((j-1)*theta);
            resultant_v(i, 2) = resultant_v(i, 2) + rel_vel_t((i-1)*nd+j, 1) * sin((j-1)*theta);
        end
    end
    for i = 1 : nc
        vel_mag = sqrt(resultant_v(i, 1)^2 + resultant_v(i, 2)^2);
        fric_mag = sqrt(resultant_f(i, 1)^2 + resultant_f(i, 2)^2);
        if vel_mag > 1e-6 && fric_mag > 1e-6
          % sliding case
          vel_dir = resultant_v(i, :) / vel_mag;
          fric_dir = resultant_f(i, :) / fric_mag;
          slide_err_angle = (1/pi)*(acos(vel_dir*fric_dir') - pi);
          slide_err_mag   = norm(U(i, i) * pn(i, 1) - fric_mag );
        else
          % sticking case
          stick_err = min(U(i, i) * pn(i, 1) - fric_mag, 0); 
        end
    end
    solution.norm_err = norm_err;
    solution.slide_err_angle = slide_err_angle;
    solution.slide_err_mag = slide_err_mag;
    solution.stick_err = stick_err;
 end

% MCP prox formulation  
if  strcmp(solver, 'fixed_point')
    obj = mlcp_fixed_point(obj);
    solution = obj.solution;

% mNCP prox formulation with PGS  AND with warm start
elseif strcmp(solver, 'fixed_point_pgs_w')
    obj.warmstart = 'quadprogram';
    obj = mncp_fixed_point_pgs(obj);
    solution = obj.solution;

% mNCP prox formulation with PGS  AND without warm start
elseif strcmp(solver, 'fixed_point_pgs_wo')
    obj.warmstart = 'nowarm';
    obj = mncp_fixed_point_pgs(obj);
    solution = obj.solution;

% mNCP prox formulation with blocked PGS  AND with warm start
elseif strcmp(solver, 'fixed_point_blockpgs_w')
    obj.warmstart = 'quadprogram';
    obj = mncp_fixed_point(obj);
    solution = obj.solution;

% mNCP prox formulation with blocked PGS  AND without warm start
elseif strcmp(solver, 'fixed_point_blockpgs_wo')
    obj.warmstart = 'nowarm';
    obj = mncp_fixed_point(obj);
    solution = obj.solution;

% LCP PGS
elseif strcmp(solver, 'PGS')
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
    solution.z = x;
    solution.err = err;
    solution.iter = iter;

% LCP Fischer - Newton 
elseif strcmp(solver, 'FischerNewton')
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
    solution.z = x;
    solution.err = err;
    solution.iter = iter;
end

end

