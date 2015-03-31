
function solution = wrap_LCP_fixed_point( obj )
% work on A to make sure that A(i, i) not equal to 0
    tic;
    solution = LCP_fixed_point(obj);
    solution.solve_time = toc; 
end

