
function solution = wrap_MlcpFixedPoint( obj )
    tic;
    obj.solution = mlcp_fixed_point(obj);
    solution = obj.solution;
    solution.solve_time = toc;
end