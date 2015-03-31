% This is the fixed_point iteration method in the form with the output as
% [z, err]
function [pn_ellp1, pf_ellp1] = ffixed_point(M, Gn, Gf, MinvPext, h, PSI, NU, num_contacts)
 
MinvGn = M \ Gn;
MinvGf = M \ Gf;

%% initialize the pn pf pn_ellp1 pf_ellp1
pn = zeros(num_contacts, 1);
pf = zeros(num_contacts*2, 1);
pn_ellp1 = zeros(num_contacts, 1);
pf_ellp1 = zeros(num_contacts*2, 1);


r = 0.5;         % the parameter r
iter = 1;        % the iteration inside the fixed_point iteration
max_iter = 10000; % the maximum number of iteration steps
flag = 0;        % to mark the convergence of pn
flag1 = 1;       % to mark the convergence of pf


%% Loop over this function until the normal force and frictional force both converge
%while (iter<max_iter && (flag == 0 && flag1 == 0))
while (iter<max_iter && flag == 0)    
    % first make the normal force converge
  %  while(flag == 0)
        rhon = PSI/h + Gn'*(NU + MinvGn*pn + MinvGf*pf + MinvPext);
        pn_temp = pn - r*rhon;
        pn_temp(pn_temp<0) = 0;
        subtra = norm(pn - pn_temp);
        Index = subtra>1e-3;
        if isempty(Index)
            flag = 1;
            pn_ellp1 = pn_temp;
            disp('The flag for pn is 1');
            pause;
        else
            pn = pn_temp;
        end
  %  end
    
    % this is to make the tangential force converge
%     while(flag1 == 0)
%         disp('This is pf ');
%         rhof = Gf'*(NU + MinvGn*pn + MinvGf*pf + MinvPext);
%         pf_temp = pf - r*rhof;
%         for i = 1 : num_contacts
%             for j = 0 : 1 % the two perpendicular frictional force on the tangential plane
%                 if pf_temp(i*2-j) > U(i, i) * pn_temp(i)
%                     pf_temp(i*2-j) = U(i, i) * pn_temp(i);
%                 end
%                 if pf_temp(i*2-j) < -U(i, i) * pn_temp(i)
%                     pf_temp(i*2-j) = -U(i, i) * pn_temp(i);
%                 end
%             end
%         end
%         subtra_f = norm(pf - pf_temp);
%         Index_f = subtra_f > 1e-2;
%         
%         if isempty(Index_f)
%             flag1 = 1;
%             pf_ellp1 = pf_temp;
%             disp('The flag for pf is 1');
%         else
%             pf = pf_temp;
%         end
%     end
    
    % update the iteration steps
    iter = iter + 1;
   % disp(iter);
end
end
