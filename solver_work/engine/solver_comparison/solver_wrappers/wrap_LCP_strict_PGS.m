function solution = wrap_LCP_strict_PGS( obj )
    tic;
    solution = LCP_strict_PGS(obj);
    solution.solve_time = toc; 
end

