%% fixed_point working version
% function[pn, pf, err] = fixed_point(M, Gn, Gf, MinvPext, h, PSI, NU, num_contacts, U)
function sim = fixed_point(sim)
M = sim.dynamics.M;
Gn = sim.dynamics.Gn;
Gf = sim.dynamics.Gf; 
h = sim.h;
FX = sim.dynamics.FX;
MinvPext = M \ FX*h;
PSI = sim.dynamics.PSI;
NU = sim.dynamics.NU;

num_contacts = length(PSI);
U = sim.dynamics.U;
MinvGn = M \ Gn;
pn = zeros(num_contacts, 1);
r =1e2;         % the parameter r
iter = 1;        % the iteration inside the fixed_point iteration
max_iter = 10000; % the maximum number of iteration steps
flag = 0;       % to mark the convergence of pn
%flag2 = 0;       % to mark the convergence of pf
err = 1e5;

while (iter<max_iter && flag == 0)
    rhon = PSI/h + Gn'*(NU + MinvGn*pn + MinvPext);
    pn_temp = pn - r*rhon;
    pn_temp(pn_temp<0) = 0;
    % to test the convergence criteria here 
    subtra = norm(pn - pn_temp);
    Index = subtra>1e-3;  
    
    if isempty(Index)  % if pn converged  
        err = PSI' *pn; % to make sure the converged pn is correst and
      %  fprintf('The error is: %d\n', err);% and satisfy the non-penetration constraint
        if(err > 1e-4)
            pn = zeros(num_contacts, 1);
        else
            flag = 1;
            pn = pn_temp;
        %    fprintf('The error is:  %d\n', err);
        end
    else
        pn = pn_temp; % continue iteration 
    end
    
    
    %pn(pn>1e4) = zeros(length(find(pn>1e4)),1);
    iter = iter + 1; 
    if iter == max_iter
      %  fprintf('The maximum iteration step has been reached \n');
    end
end 
% pf = zeros(num_contacts*2, 1);

% the convergence of the frictional force
% MinvGf = M \ Gf;
% iter1 = 1;
% max_iter1 = 10000;
% flag1 = 0;
% r1 = 0.01;
pf = zeros(num_contacts*2, 1);
% 
% while (iter1 < max_iter1 && flag1 == 0)
%     rhof = Gf' * (NU + MinvGn*pn + MinvGf*pf + MinvPext);
%     pf_temp = pf - r1*rhof;
%     for i=1:num_contacts
%         for j = 0:1
%             if pf_temp(i*2-j) > U(i, i) * pn_temp(i)
%                 pf_temp(i*2-j) = U(i, i) * pn_temp(i);
%             end
%             if pf_temp(i*2-j) < -U(i, i) * pn_temp(i)
%                 pf_temp(i*2-j) = -U(i, i) * pn_temp(i);
%             end
%         end
%     end
%     subtra_f = norm(pf - pf_temp);
%     Index_f = subtra_f > 1e-2;
%     if isempty(Index_f)
%         flag1 = 1;
%         pf = pf_temp;
%     else 
%         pf  = pf_temp;
%     end
%     iter1 = iter1 + 1;
% end
sim.err = err;
sim.iter = iter;
end