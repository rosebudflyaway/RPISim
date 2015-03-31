
function solution = wrap_NcpFixedPoint( obj )
% This is the NCP model, first iterate on Pn (of all the contacts)
% Then iterate on the Pf (for all the contacts)
% Done
   tic;    
   obj.solution = mncp_fixed_point(obj);
   solution = obj.solution;
   solution.solve_time = toc; 
end