function solution = wrap_LcpMinmapNewton( obj )
%     A = obj.A_LCP;
%     b = obj.b_LCP;
%     z0 = obj.z0_LCP; 
    tic;
    solution = minmap_newton(obj);
    solution.solve_time = toc; 
%     solution.z = z;
%     solution.err = err;
%     solution.iter = iter;    
end

