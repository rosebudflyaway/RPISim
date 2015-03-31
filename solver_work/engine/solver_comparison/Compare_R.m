
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: 
%   solverHandles    - A cell array of the function names as STRINGS.
%
%   problemFile      - A string representing the .h5 file containing the problems. 
%
%   problem_indicies - (optional) An array of indices to use in the comparison.  
%                      If not specified, the entire problemFile will be used.
%
% OUTPUT:
%   results - A struct containing the results of comparison.  

function results = Compare_R(solverHandles, problemFile, ErrorParams, problem_indicies, tunable_params, source)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Process the function arguments
    DATA = h5load(problemFile, source, problem_indicies);
    FormulationType = tunable_params.FormulationType; 
    formulation = tunable_params.formulation;
  
    Rvalues = tunable_params.r;
     % The following is to keep consistent with the TESTCompareSolvers.m
    TuneParams = struct();
    TuneParams.max_iter = tunable_params.max_iter;
    TuneParams.tol = tunable_params.tol;
    TuneParams.SaveOrNot = tunable_params.SaveOrNot;
    TuneParams.alpha = tunable_params.alpha;
    NUM_PROBLEMS = length(fieldnames(DATA));

    fprintf('done.\n'); 
    
    if iscell(solverHandles)
        if ischar(solverHandles{1})
            SOLVER_HANDLES = cell(length(solverHandles),1); 
            for i=1:length(solverHandles)
                s = solverHandles{i}; 
                if length(s) >= 4 && strcmp(s(1:4),'wrap')
                    SOLVER_HANDLES{i} = str2func(solverHandles{i}); % If they used the wrapper function
                else 
                    if strcmpi(solverHandles{i},'MLCPproxPGS')
                        SOLVER_HANDLES{i} = @wrap_MlcpFixedPoint;
                    elseif strcmpi(solverHandles{i}, 'MLCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_MLCP_strict_PGS;    
                    elseif strcmpi(solverHandles{i}, 'LCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_LCP_strict_PGS;
                    elseif strcmpi(solverHandles{i}, 'LCP_fixed_point')
                        SOLVER_HANDLES{i} = @wrap_LCP_fixed_point;
                    elseif strcmpi(solverHandles{i},'NCPproxPGS')
                        SOLVER_HANDLES{i} = @wrap_NcpFixedPoint;
                    elseif strcmpi(solverHandles{i}, 'NCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_NCP_strict_PGS;
                    elseif strcmpi(solverHandles{i}, 'NCP_strict_PGS2')
                        SOLVER_HANDLES{i} = @wrap_NCP_strict_PGS2;
                    end
                end
            end
        elseif isa(solverHandles{1},'function_handle')
            SOLVER_HANDLES = solverHandles;
        else
            error('Unrecognized format of "solverHandles."  Check the description in compareSolvers.m');
        end
    else
        error('Unrecognized format of "solverHandles."  It should be a cell array.');
    end
    NUM_SOLVERS = length(SOLVER_HANDLES); 
    PROBNAMES = fieldnames(DATA); 
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialize variables 
    results.description = ['(' date '): Comparison of ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS) ' problems.  '];
    NUM_R = length(Rvalues);
    % These will store the results at every iteration
    iterations = zeros(NUM_R, NUM_PROBLEMS);
    solve_time = zeros(NUM_R, NUM_PROBLEMS);   % TODO: The solve_time values are not accurate inside the wrappers
    final_absolute_error = zeros(NUM_R, NUM_PROBLEMS);
    total_error = zeros(1000,NUM_R, NUM_PROBLEMS);
    friction_error = zeros(1000,NUM_R, NUM_PROBLEMS);
    friction_FBerror = zeros(1000, NUM_R, NUM_PROBLEMS);
    
    normal_error = zeros(1000, NUM_R, NUM_PROBLEMS);
    normal_FBerror = zeros(1000, NUM_R, NUM_PROBLEMS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% Main loop: passes problems to solvers and records stats
    fprintf(['Running ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS)  ' problems...']);
    fprintf('       %% complete \n');
    progress = 0; 
    % For each r
    for pr = 1:NUM_R
        %%%%%%%%%%%%%%%%%%%%%%% Main loop on r %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for problem = 1 : NUM_PROBLEMS
            P = DATA.(PROBNAMES{problem});
            P.solver.errmetric = struct();
            P.solver.errmetric = ErrorParams;
            TuneParams.r = Rvalues(pr);
            P.solver.TuneParams = struct();
            P.solver.TuneParams = TuneParams;
            P.source  = source;
            P.num_fricdirs = 4;
            % we won't run the problem with no contacts
            if isfield(P, 'contacts')
                if ~strcmp(FormulationType , 'particleDynamics')
                    if ~strcmp(formulation, 'mNCP')  % Linearized formulations
                        P.dynamics = PreDynamicsWFormulation(P,'');
                        [A b z0] = makeLCP(P);
                        P.dynamics.A_LCP = A; P.dynamics.b_LCP = b; P.dynamics.z0_LCP = z0;
                        [A b z0] = makeMCP(P);
                        P.dynamics.A_MCP = A; P.dynamics.b_MCP = b; P.dynamics.z0_MCP = z0;
                    else
                        P.Ndynamics = PreDynamicsWFormulation(P,'mNCP');     % Non-linear formulations (mNCP wrappers should use this)
                    end
                else
                    if ~strcmp(formulation, 'mNCP')  % Linearized formulations
                        P.dynamics = PreParticleDynamics(P, formulation);
                    else
                        P.Ndynamics = PreParticleDynamics(P, 'mNCP');
                    end
                end
                % To make it simple, we will use one solver each time.
                % Also this will be wise since different solvers require
                % different r values. So the range might vary a lot
                
                % Solve % we have only one solver
                solution = feval(SOLVER_HANDLES{1}, P);
                
                % Store results of the corresponding solver of a problem
                iterations(pr,problem) = solution.iterations;
                solve_time(pr,problem) = solution.solve_time;
                total_error(1:solution.iterations, pr, problem) = solution.total_error(1:solution.iterations);
                friction_error(1:solution.iterations, pr, problem) = solution.friction_error(1:solution.iterations);
                friction_FBerror(1:solution.iterations, pr, problem) = solution.fricFBerror(1:solution.iterations);
                normal_error(1:solution.iterations, pr, problem) = solution.normal_error(1:solution.iterations);
                normal_FBerror(1:solution.iterations, pr, problem) = solution.normFBerror(1:solution.iterations);
                final_absolute_error(pr, problem) = abs( solution.total_error(solution.iterations));
                
                percentDone = 100*problem/NUM_PROBLEMS;
                if percentDone > progress+10 && percentDone < 100
                    fprintf([repmat('\b',1,12) num2str(round(percentDone)) '%% complete']);
                    progress = percentDone;
                end
            end
        end % for problem = 1 : NUM_PROBLEMS
    end % for pr = 1 : NUM_R 
    fprintf([repmat('\b',1,12) '100%% done.\n']);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Gather results to return
    results.iterations = iterations; 
    results.solve_time = solve_time;
    results.total_error = total_error; 
    results.friction_error = friction_error;
    results.fricFBerror = friction_FBerror;
    results.normal_error = normal_error; 
    results.normFBerror = normal_FBerror;
    
    results.absolute_error = final_absolute_error; 
    results.savePlot =  tunable_params.SaveOrNot;
    results.out = datestr(now,'mm_dd_HH_MM');
    if results.savePlot
        mkdir(results.out);
    end
    
    %% Plot the convergence of each Rvalue over all problems
    r_range = 1:NUM_R;  % all the r values
    
    avg_tot = zeros(NUM_PROBLEMS, 1000);    
    
    for problem = 1 : NUM_PROBLEMS
        tot_err = total_error(1:max(iterations(r_range, problem)), r_range, problem)';
        avg_tot(problem, 1:size(tot_err, 2)) = mean(tot_err, 1);
    end
    
    color2use = hsv(NUM_R);
    % Make up the string array for the lengeds
    r_legend = cell(1, NUM_R);
    for k = 1 : NUM_R
        r_legend{k} = strcat('r = ', num2str(Rvalues(k)));
    end 
   % sn = cell{NUM_R, 1};
    count = 1;
    for problem = 1 : NUM_PROBLEMS
        figure;
        total_error_single_solver = tot_err(:, :, problem)'; %(iterations by NUM_R)
        for j = 1 : NUM_R
             x_length = length(total_error_single_solver(:, j));
             semilogy((0:x_length-1), total_error_single_solver(:, j), '-', 'Color', color2use(j, :));
             hold on;
             %legend(r_legend{j});
        end
        legend(r_legend, 'FontSize', 12);
        xlabel('Solver Iteration', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
        ylabel('total error', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
        title(strcat('Fixed-point of prox with relaxation =',  num2str(TuneParams.alpha), ' and tol = ',num2str(TuneParams.tol)), 'FontSize', 14,  'FontWeight', 'bold');
        
        hold off;
        count = count + 1;
    end   
end  
     
    
    
    
    
    
    