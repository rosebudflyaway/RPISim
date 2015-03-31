
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

function results = compareSolverManyProblem(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Process the function arguments
    fprintf('\nLoading hdf5 file...   ');
    
    DATA = h5load(problemFile, source, problem_indicies);
    formulation = TuneParams.formulation;
    FormulationType = TuneParams.FormulationType; 
    tolerance = TuneParams.tol;
    r = TuneParams.r;
    SaveOrNot = TuneParams.SaveOrNot;
  
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
    
    NUM_SOLVERS = length(SOLVER_HANDLES); 
    PROBNAMES = fieldnames(DATA); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialize variables 
    results.description = ['(' date '): Comparison of ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS) ' problems.  ']; 
    % These will store the results at every iteration
    iterations = zeros(NUM_PROBLEMS, NUM_SOLVERS);
    solve_time = zeros(NUM_PROBLEMS, NUM_SOLVERS);   % TODO: The solve_time values are not accurate inside the wrappers
    final_absolute_error = zeros(NUM_PROBLEMS, NUM_SOLVERS);   % take log10 of the error at the last iteration of each solver 
%     total_error = zeros(1000,NUM_PROBLEMS, NUM_SOLVERS);
%     friction_error = zeros(1000,NUM_PROBLEMS, NUM_SOLVERS);
%     friction_FBerror = zeros(1000, NUM_PROBLEMS, NUM_SOLVERS); 
%     
%     normal_error = zeros(1000, NUM_PROBLEMS, NUM_SOLVERS);
%     normal_FBerror = zeros(1000, NUM_PROBLEMS, NUM_SOLVERS);
    
    total_error = zeros(40,NUM_PROBLEMS, NUM_SOLVERS);
    friction_error = zeros(40,NUM_PROBLEMS, NUM_SOLVERS);
    friction_FBerror = zeros(40, NUM_PROBLEMS, NUM_SOLVERS); 
    
    normal_error = zeros(40, NUM_PROBLEMS, NUM_SOLVERS);
    normal_FBerror = zeros(40, NUM_PROBLEMS, NUM_SOLVERS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Main loop: passes problems to solvers and records stats
    fprintf(['Running ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS)  ' problems...']);
    fprintf('       %% complete \n');
    progress = 0; 
    % For each problem
    for problem=1:NUM_PROBLEMS
        P = DATA.(PROBNAMES{problem});
        % SET THE R VALUE FOR PROX method
        P.solver.Rvalue = r;
        P.solver.tolerance = tolerance;
        P.solver.errmetric = struct();
        P.solver.errmetric = ErrorParams;
        P.solver.TuneParams = struct();
        P.solver.TuneParams = TuneParams;
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
                % Solve
                solution = feval(SOLVER_HANDLES{solver}, P);   
                % Store results of the corresponding solver of a problem
                iterations(problem,solver) = solution.iterations;
                solve_time(problem,solver) = solution.solve_time;
                total_error(1:solution.iterations,problem,solver) = solution.total_error(1:solution.iterations);       
                friction_error(1:solution.iterations,problem,solver) = solution.friction_error(1:solution.iterations);                 
                friction_FBerror(1:solution.iterations, problem, solver) = solution.fricFBerror(1:solution.iterations);               
                normal_error(1:solution.iterations, problem, solver) = solution.normal_error(1:solution.iterations);
                normal_FBerror(1:solution.iterations, problem, solver) = solution.normFBerror(1:solution.iterations);
                final_absolute_error(problem,solver) = abs( solution.total_error(solution.iterations));
            end      
            percentDone = 100*problem/NUM_PROBLEMS;
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
    
    % pause
    FigureCount = 1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Plot results over the problem
        
    color2use = hsv(8);
    % TOTAL ERROR / SOLVER TIME  versus PROBLEM
    % total error versus the # of problems 
    solverHandles = regexprep(solverHandles,'_',' '); % regular expression
    %set(0,'DefaultAxesFontSize',14);
%     figure;
%         plot(solve_time, 'LineWidth', 2);           % TODO: match the line formatting all the way through 
%         grid on;
%         xlabel('Problem Number', 'FontSize', 14);
%         ylabel('Solver Time', 'FontSize', 14);
%         title('The solver time', 'FontSize', 14);
%         legend(solverHandles, 'FontSize', 14 );
%         if results.savePlot
%             print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/time_error.eps'));
%             FigureCount = FigureCount + 1;
%         end
%     
%     figure;
%     for solver = 1 : NUM_SOLVERS
%         current_abs_err = final_absolute_error(:, solver);
%         semilogy(current_abs_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Problem number',  'FontSize', 14);
%         ylabel('absolute error',  'FontSize', 14)
%         hold all;
%         title('Absolute error at the last iteration', 'FontSize', 14);
%     end
%     legend(solverHandles, 'FontSize', 14);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Plot results over the iterations in one problem / averaged by all the iterations / problems
    
    %% Plot the convergence of each solver over all problems 
    p_range = 1:NUM_PROBLEMS;  % all problem
    
%     avg_tot = zeros(NUM_SOLVERS, 1000);
%     
%     avg_normal = zeros(NUM_SOLVERS, 1000);
%     avg_normFB = zeros(NUM_SOLVERS, 1000);
%     avg_frictional = zeros(NUM_SOLVERS, 1000);
%     avg_frictionalFB = zeros(NUM_SOLVERS, 1000);
    
    avg_tot = zeros(NUM_SOLVERS, 40);
    
    avg_normal = zeros(NUM_SOLVERS, 40);
    avg_normFB = zeros(NUM_SOLVERS, 40);
    avg_frictional = zeros(NUM_SOLVERS, 40);
    avg_frictionalFB = zeros(NUM_SOLVERS, 40);
    
    for solver = 1:NUM_SOLVERS
        tot_err = total_error(1:max(iterations(p_range,solver)), p_range, solver)'; % to use mean(tot_err, 1);
        norm_err = normal_error(1:max(iterations(p_range,solver)), p_range, solver)';
        normFBerr = normal_FBerror(1:max(iterations(p_range, solver)), p_range, solver)';
        
        fric_err = friction_error(1:max(iterations(p_range,solver)), p_range, solver)';
        fricFBerr = friction_FBerror(1:max(iterations(p_range, solver)), p_range, solver)';
        
        % THE SIZE OF AVG( by mean() ) IS  1 by NUM_ITERATIONS
        avg_tot(solver, 1:size(tot_err, 2)) = mean(tot_err, 1);
        avg_normal(solver, 1:size(tot_err, 2)) = mean(norm_err, 1);
        avg_normFB(solver, 1:size(tot_err, 2)) = mean(normFBerr, 1);
        avg_frictional(solver, 1:size(tot_err, 2)) = mean(fric_err, 1);
        avg_frictionalFB(solver, 1:size(tot_err, 2)) = mean(fricFBerr, 1);
    end
    
    color2use = hsv(NUM_PROBLEMS);
   % don't take average over the tot_err;
   % There is a figure for each solver and on each plot, there are
   % NUM_PROBLEMS legends corresponding to each problem. 
   for solver = 1 : NUM_SOLVERS
       figure;
       total_error_single_solver =tot_err(:, :, solver)';  % (iterations by problems) 
       for j = 1 : NUM_PROBLEMS
           semilogy(total_error_single_solver(:, j), '-o', 'LineWidth', 2, 'Color', color2use(j, :));           
           hold on;
       end
       xlabel('Solver Iteration', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
       ylabel('total error', 'LineWidth', 2, 'FontSize', 14,  'FontWeight','bold');
       hold off;
   end
%     average of the tot_err
%     figure;
%     for solver = 1 : NUM_SOLVERS
%         current_tot_err = avg_tot(solver, :);
%         current_tot_err = current_tot_err(current_tot_err > 0);
%         semilogy(current_tot_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Solver Iteraion', 'LineWidth', 2, 'FontSize', 16);
%         ylabel('total error', 'LineWidth', 2, 'FontSize', 16)
%         hold all;
%         title('Average total error over all iterations', 'FontSize', 16);
%     end
%     legend(solverHandles, 'FontSize', 14);
%     if results.savePlot
%         print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/totalErr.eps'));
%         FigureCount = FigureCount + 1;
%     end
%     hold off;



%     figure;
%     % average of the normal_error (physical error)
%     for solver = 1 : NUM_SOLVERS
%         current_norm_err = avg_normal(solver, :);
%         current_norm_err = current_norm_err(current_norm_err > 0);
%         semilogy(current_norm_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Solver Iteraion', 'LineWidth', 2, 'FontSize', 16);
%         ylabel('physical normal error', 'LineWidth', 2, 'FontSize', 16);
%         hold all;
%         title('Average physical normal error over all iterations', 'FontSize', 16);
%     end
%     hold off; 
%     legend(solverHandles, 'FontSize', 14);   
%     if results.savePlot
%         print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/normalErr.eps'));
%         FigureCount = FigureCount + 1;
%     end
%    
%     
%     figure;
%     % average of the normal_FB_error
%     for solver = 1 : NUM_SOLVERS
%         current_nFB_err = avg_normFB(solver, :);
%         current_nFB_err = current_nFB_err(current_nFB_err > 0);
%         semilogy(current_nFB_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Solver Iteraion', 'LineWidth', 2, 'FontSize', 16);
%         ylabel('FB normal error', 'LineWidth', 2, 'FontSize', 16);
%         hold all;
%         title('Average FB normal error over all iterations', 'FontSize', 16);
%     end
%     legend(solverHandles, 'FontSize', 14);
%     
%     if results.savePlot
%         print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/normFBErr.eps'));
%         FigureCount = FigureCount + 1;
%     end
%     hold off;
%     
%     figure;
%     % average of the frictional error (physical error)
%     for solver = 1 : NUM_SOLVERS
%         current_frc_err = avg_frictional(solver, :);
%         current_frc_err = current_frc_err(current_frc_err > 0);
%         semilogy(current_frc_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Solver Iteraion', 'LineWidth', 2, 'FontSize', 16);
%         ylabel('physical frictional error', 'LineWidth', 2, 'FontSize', 16);
%         hold all;
%         title('Average physical frictional error over all iterations', 'FontSize', 16);
%     end
%     hold off;
%     legend(solverHandles, 'FontSize', 14);
%     
%     if results.savePlot
%         print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/fricErr.eps'));
%         FigureCount = FigureCount + 1;
%     end
%  
%     
%     figure;
%     % average of the frictional FB error 
%     for solver = 1 : NUM_SOLVERS
%         current_fNB_err = avg_frictionalFB(solver, :);
%         current_fNB_err = current_fNB_err(current_fNB_err > 0);
%         semilogy(current_fNB_err, 'LineWidth', 2, 'Color', color2use(solver, :));
%         xlabel('Solver Iteraion', 'LineWidth', 2, 'FontSize', 16);
%         ylabel('FB frictional error', 'LineWidth', 2, 'FontSize', 16);
%         hold all;
%         title('Average FB frictional error over all iterations', 'FontSize', 16);
%     end
%     legend(solverHandles, 'FontSize', 14);
%     if results.savePlot
%         print(strcat('-f', num2str(FigureCount)),'-depsc2',strcat(results.out, '/fricFBerr.eps'));
%         FigureCount = FigureCount + 1;
%     end
%     hold off;
end
    
    
    