
function solution = wrap_mpgs( obj )
% work on A to make sure that A(i, i) not equal to 0
    tic;
    solution = pgs(obj);
    solution.solve_time = toc; 
   % solution.z = z;
   % solution.err = err;
   % solution.iter = iter;
end

