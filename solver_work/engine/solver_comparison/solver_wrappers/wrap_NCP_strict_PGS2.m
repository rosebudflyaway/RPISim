function solution = wrap_NCP_strict_PGS2( obj )
% This is to ignore convergence on the sliding speed s
% Only iterate  the normal impulse Pn and frictional impulse Pf 
% Don't consider the sliding speed s in the NCP model

   tic;    
   obj.solution = mncp_strict_pgs2(obj);
   solution = obj.solution;
   solution.solve_time = toc; 
end