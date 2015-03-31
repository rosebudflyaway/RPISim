function solution = wrap_MLCP_strict_PGS( obj )
% The mlcp_strict_pgs.m solves the problem row by row, equation by equation
    tic;
    obj.solution = mlcp_strict_pgs(obj);
    solution = obj.solution;
    solution.solve_time = toc;
end