
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: 
%   solverHandles    - A cell array of the function names as STRINGS.

%   source = 1;   % 1 --- to use the h5loader 
                  % 2 --- to use the HDF5loader & PROCESS & CHANGEINCONSISTENCY
%
%   problemFile      - A string representing the .h5 file containing the problems. 
%
%   ErrorParams      - parameters for different error metrics
%
%   problem_indicies - (optional) An array of indices to use in the comparison.  
%                      If not specified, the entire problemFile will be used.

%  The fields of TuneParams
%
%   tol              - (optional) tolerance of the solvers
% 
%   r                - (optional) r parameter in the prox function 
%
%   SaveOrNot        - (optional) with true  to save the plot in a folder
% OUTPUT:
%   results - A struct containing the results of comparison.  

function results = CompareManySolversOneProb(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Process the function arguments
    fprintf('\nLoading hdf5 file...   ');
    
    DATA = h5load(problemFile, source, problem_indicies);
    formulation = TuneParams.formulation;
    FormulationType = TuneParams.FormulationType; 
    fprintf('done.\n'); 
       
    tolerance = TuneParams.tol;
    Rvalues = TuneParams.rvalues;
    Lambda1 = TuneParams.lambdaONE;
    Lambda2 = TuneParams.lambdaTWO;
    SaveOrNot = TuneParams.SaveOrNot;
  
     
    if iscell(solverHandles)
        if ischar(solverHandles{1})
            SOLVER_HANDLES = cell(length(solverHandles),1); 
            for i=1:length(solverHandles)
                s = solverHandles{i}; 
                if length(s) >= 4 && strcmp(s(1:4),'wrap')
                    SOLVER_HANDLES{i} = str2func(solverHandles{i}); % If they used the wrapper function
                else
                    if strcmpi(solverHandles{i},'Lemke')
                        SOLVER_HANDLES{i} = @wrap_Lemke;
                    elseif strcmpi(solverHandles{i},'PATH')
                        SOLVER_HANDLES{i} = @wrap_PATH;
                    elseif strcmpi(solverHandles{i},'MLCPproxPGS')
                        SOLVER_HANDLES{i} = @wrap_MlcpFixedPoint; 
                    elseif strcmpi(solverHandles{i}, 'MLCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_MLCP_strict_PGS;
                    elseif strcmpi(solverHandles{i}, 'LCP_fixed_point')
                        SOLVER_HANDLES{i} = @wrap_LCP_fixed_point;
                    elseif strcmpi(solverHandles{i}, 'LCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_LCP_strict_PGS;
                    elseif strcmpi(solverHandles{i},'NCPproxPGS')
                        SOLVER_HANDLES{i} = @wrap_NcpFixedPoint;
                    elseif strcmpi(solverHandles{i}, 'NCP_strict_PGS')
                        SOLVER_HANDLES{i} = @wrap_NCP_strict_PGS;
                    elseif strcmpi(solverHandles{i}, 'NCP_strict_PGS2')
                        SOLVER_HANDLES{i} = @wrap_NCP_strict_PGS2;
                    elseif strcmpi(solverHandles{i},'PGS')
                        SOLVER_HANDLES{i} = @wrap_mpgs;
                    elseif strcmpi(solverHandles{i},'FischerNewton')
                        SOLVER_HANDLES{i} = @wrap_LcpFischerNewton;
                    elseif strcmpi(solverHandles{i}, 'interior_point')
                        SOLVER_HANDLES{i} = @wrap_LcpInteriorPoint;
                    elseif strcmpi(solverHandles{i}, 'minmap_newton')
                        SOLVER_HANDLES{i} = @wrap_LcpMinmapNewton;
                    elseif strcmpi(solverHandles{i}, 'psor')
                        SOLVER_HANDLES{i} = @wrap_LcpPSOR;
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
    
    NUM_PROBLEMS = length(fieldnames(DATA));
    NUM_SOLVERS = length(SOLVER_HANDLES); 
    PROBNAMES = fieldnames(DATA); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialize variables 
    results.description = ['(' date '): Comparison of ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS) ' problems.  ']; 
    % These will store the results at every iteration
    iterations = zeros(NUM_SOLVERS, NUM_PROBLEMS);
    solve_time = zeros( NUM_SOLVERS, NUM_PROBLEMS);   % TODO: The solve_time values are not accurate inside the wrappers
    final_absolute_error = zeros(NUM_SOLVERS, NUM_PROBLEMS);   % take log10 of the error at the last iteration of each solver 
    total_error = zeros(1000, NUM_SOLVERS, NUM_PROBLEMS);
    friction_error = zeros(1000, NUM_SOLVERS, NUM_PROBLEMS);
    friction_FBerror = zeros(1000, NUM_SOLVERS, NUM_PROBLEMS); 
    
    normal_error = zeros(1000, NUM_SOLVERS, NUM_PROBLEMS);
    normal_FBerror = zeros(1000, NUM_SOLVERS, NUM_PROBLEMS);
    
%     total_error = zeros(40,NUM_PROBLEMS, NUM_SOLVERS);
%     friction_error = zeros(40,NUM_PROBLEMS, NUM_SOLVERS);
%     friction_FBerror = zeros(40, NUM_PROBLEMS, NUM_SOLVERS); 
%     
%     normal_error = zeros(40, NUM_PROBLEMS, NUM_SOLVERS);
%     normal_FBerror = zeros(40, NUM_PROBLEMS, NUM_SOLVERS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Main loop: passes problems to solvers and records stats
    fprintf(['Running ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS)  ' problems...']);
    fprintf('       %% complete \n');
    progress = 0; 
    % For each problem
    for problem=1:NUM_PROBLEMS
        P = DATA.(PROBNAMES{problem});   
        P.num_fricdirs = 4;
        P.source = source;
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
            % For each solver
            for solver=1:NUM_SOLVERS
                % SET THE R VALUE FOR PROX method
                P.solver.Rvalue = Rvalues(solver);
                P.solver.tolerance = tolerance;
                P.solver.errmetric = struct();
                P.solver.errmetric = ErrorParams;
                TuneParams.r = Rvalues(solver);
                TuneParams.lambda1 = Lambda1(solver);
                TuneParams.lambda2 = Lambda2(solver);
                P.solver.TuneParams = struct();
                P.solver.TuneParams = TuneParams;             
                % Solve
                solution = feval(SOLVER_HANDLES{solver}, P);   
                % Store results of the corresponding solver of a problem
                iterations(solver, problem) = solution.iterations;
                solve_time(solver, problem) = solution.solve_time;
                total_error(1:solution.iterations,solver, problem) = solution.total_error(1:solution.iterations);       
                friction_error(1:solution.iterations,solver, problem) = solution.friction_error(1:solution.iterations);                 
                friction_FBerror(1:solution.iterations, solver, problem) = solution.fricFBerror(1:solution.iterations);               
                normal_error(1:solution.iterations, solver, problem) = solution.normal_error(1:solution.iterations);
                normal_FBerror(1:solution.iterations, solver, problem) = solution.normFBerror(1:solution.iterations);
                final_absolute_error(solver, problem) = abs( solution.total_error(solution.iterations));
            end      
            percentDone = 100*solver/NUM_SOLVERS;
            if percentDone > progress+10 && percentDone < 100
                fprintf([repmat('\b',1,12) num2str(round(percentDone)) '%% complete']);
                progress = percentDone;
            end
        end
    end
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
    results.savePlot = SaveOrNot;
    results.out = datestr(now,'mm_dd_HH_MM');
    if results.savePlot
        mkdir(results.out);
    end
    
 
    %% Plot results over the iterations in one problem / averaged by all the iterations / problems
    
    %% Plot the convergence of each solver over all problems 
    s_range = 1:NUM_SOLVERS;  % all problem    
    avg_tot = zeros(NUM_PROBLEMS, 1000);
    for problem = 1 : NUM_PROBLEMS
        tot_err = total_error(1:max(iterations(s_range, problem)), s_range, problem)';
        avg_tot(problem, 1:size(tot_err, 2)) = mean(tot_err, 1);
    end
    
    color2use = hsv(NUM_SOLVERS);
    %s_legend = {'Lemke', 'PATH', 'MLCP prox fp', 'LCP strictPGS fp (r = 0.4)', ...
    %    'LCP strictPGS fp (r = 0.7)',  'LCP strictPGS fp (r = 1.0)',  'PGS', 'Newton CCK (\lambda_1 = 0.2)'...
    %    'Newton CCK (\lambda_1 = 0.8)', 'interior point', 'Newton min', 'PSOR'};
    
    s_legend = {'Lemke', 'PATH', 'LCP prox fp', 'MLCP prox fp', 'LCP strictPGS fp (r = 0.4)', ...
        'LCP strictPGS fp (r = 0.7)',  'LCP strictPGS fp (r = 1.0)',  'PGS', 'Newton CCK (\lambda_1 = 0.2)'...
        'Newton CCK (\lambda_1 = 0.8)', 'interior point', 'Newton min', 'PSOR'};
   % don't take average over the tot_err;
   % There is a figure for each solver and on each plot, there are
   % NUM_PROBLEMS legends corresponding to each problem. 
   for problem = 1 : NUM_PROBLEMS
       figure;
       total_error_single_problem =tot_err(:, :, problem)';  % (iterations by problems) 
       for j = 1 : NUM_SOLVERS
           if strcmp(solverHandles{j}, 'PATH')
               x_length = length(total_error_single_problem(:, j));
               total_error_single_problem(2, j) = 10^(-15);
               %semilogy((0:x_length-1), total_error_single_problem(:, j), '-*', 'LineWidth', 2, 'Color', color2use(j, :));
               semilogy((0:x_length-1), total_error_single_problem(:, j), 'b-*', 'MarkerSize', 10);
           else
               x_length = length(total_error_single_problem(:, j));
               semilogy((0:x_length-1), total_error_single_problem(:, j), '-', 'LineWidth', 2, 'Color', color2use(j, :));
           end
           hold on;
       end
       xlabel('Solver Iteration', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
       ylabel('total error', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
       title('Different solver comparison for particle problem', 'FontSize', 14, 'FontWeight', 'bold');
       legend(s_legend, 'FontSize', 12);
       hold off;
   end
end
    
    
    