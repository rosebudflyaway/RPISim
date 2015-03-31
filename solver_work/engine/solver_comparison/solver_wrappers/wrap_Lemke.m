
function solution = wrap_Lemke( obj )
%     A = obj.A_LCP;
%     b = obj.b_LCP;
%     z0 = obj.z0_LCP; 
    
%     nb = length(obj.bodies.masses);
%     nc = length(obj.contacts.depth);
%     nd = obj.dynamics.nd;
    
%     obj.dynamics.A = obj.dynamics.A_LCP;
%     obj.dynamics.b = obj.dynamics.b_LCP;
%     obj.dynamics.z0 = obj.dynamics.z0_LCP; 
    
    tic;
    solution = lemke( obj );   
    solution.solve_time = toc; 
    % the total error 
%     error = z' * (A*z + b);
%     solution.z = z;
%     solution.iter = iter;
%     % TODO: put the detailed error here: Norm, Frictional
%     solution.err = error;    
    % Handle potential errors
%     if err ~= 0
%         if size(err) == 1
%             disp(['LCP Error: ' num2str(err)]);
%         else
%             disp('LCP Error');
%         end
%     end
    
    % deal with the detailed error information
%     pn = z(1:nc, 1);
%     pf = z(nc+1:nc+nc*nd, 1);
%     s  = z(nc+nc*nd+1:nc+nc*nd+nc, 1);
%     PSI = obj.contacts.depth;
%     U = obj.dynamics.U;
%     theta = 2*pi/nd;
%     % compute the magnitude of the friction and its direction using the
%     % linearized frictions returned from the solution 
%     fricMag = zeros(nc, 1);
%     resultant_f = zeros(nc, 2);
%     for i = 1 : nc 
%         for j = 1 : nd
%             resultant_f(i, 1) = resultant_f(i, 1) + pf((i-1)*nd+j, 1) * cos((j-1)*theta);
%             resultant_f(i, 2) = resultant_f(i, 2) + pf((i-1)*nd+j, 1) * sin((j-1)*theta);
%         end
%         fricMag(i, 1) = sqrt(resultant_f(i, 1) ^ 2 + resultant_f(i, 2) ^2 );
%     end
%     MagMinusPf = U * pn - fricMag;
%     norm_err = norm(pn' * PSI);
%     fric_err = norm( s' * MagMinusPf);
%     solution.norm_err = norm_err;
%     solution.fric_err = fric_err;
end

