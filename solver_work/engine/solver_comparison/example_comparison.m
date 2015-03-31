% This short example demonstrates how to compare 
% solver statistics on a problem dataset.  
problem_file = 'logs.h5';   % The dataset of problems
problems = 12:15;           % Which problems to solve (optional)
solvers = { %'PATH'         % Which solvers to compare
            'Lemke'
            %'PGS'
            'fixed_point'
            %'fixed_point_pgs_w'
            'FischerNewton' 
            'interior_point'
            %'minmap_newton'
            };
results = compareSolvers(solvers, problem_file, problems); 
