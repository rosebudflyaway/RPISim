function solution = wrap_NCP_strict_PGS( obj )
% This is the NCP model for strict PGS 
% iterate on row by row (with the sliding speed s), or equation by equation
   tic;    
   obj.solution = mncp_strict_pgs(obj);
   solution = obj.solution;
   solution.solve_time = toc; 
end