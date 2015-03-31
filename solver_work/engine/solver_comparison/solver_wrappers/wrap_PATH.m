function solution = wrap_PATH( obj )
    A = obj.dynamics.A_MCP;
    b = obj.dynamics.b_MCP;
    z0 = obj.dynamics.z0_MCP; 
     
    nb = length(obj.dynamics.Vel) / 6;
    nc = length(obj.contacts.PSI);
    nd = obj.dynamics.nd;
    U = obj.dynamics.U;
    PSI = obj.contacts.PSI;
    
    problem_size = length(b);
    big = 10^20;
    u = big*ones(problem_size, 1);
    l = [-big * ones(6*nb, 1);
          zeros((2+nd)*nc, 1)];
    obj.solution.big = big;
    obj.solution.l = l;
    obj.solution.u = u;
    tic;
    solution = pathlcp(obj);
    solution.solve_time = toc; 
    obj.z = solution.NUnew;
 end
