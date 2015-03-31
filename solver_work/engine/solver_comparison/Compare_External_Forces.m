
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

function results = Compare_External_Forces(solverHandles, problemFile, ErrorParams, problem_indicies, tunable_params, source)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Process the function arguments
    DATA = h5load(problemFile, source, problem_indicies);
    FormulationType = tunable_params.FormulationType; 
    formulation = tunable_params.formulation;
  
    Rvalues = tunable_params.r;
    External_Forces = tunable_params.Forces;
     % The following is to keep consistent with the TESTCompareSolvers.m
    TuneParams = struct();
    TuneParams.max_iter = tunable_params.max_iter;
    TuneParams.tol = tunable_params.tol;
    TuneParams.SaveOrNot = tunable_params.SaveOrNot;
    TuneParams.alpha = tunable_params.alpha;
    %%%%%%%%% Here we only consider one problem, when the particle is going
    %%%%%%%%% to hit the wall, so the NUM_PROBLEMS = 1;
    NUM_PROBLEMS = length(fieldnames(DATA));
    
    NUM_R = length(Rvalues);
    NUM_FORCES = length(External_Forces);

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
    PROBNAMES  = fieldnames(DATA);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialize variables 
    results.description = ['(' date '): Comparison of ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS) ' problems.  '];
    % These will store the results at every iteration
    iterations = zeros(NUM_FORCES, NUM_R);
    solve_time = zeros(NUM_FORCES, NUM_R);   % TODO: The solve_time values are not accurate inside the wrappers
    final_absolute_error = zeros(NUM_FORCES, NUM_R);
    total_error = zeros(1000, NUM_FORCES, NUM_R);
    friction_error = zeros(1000, NUM_FORCES, NUM_R);
    friction_FBerror = zeros(1000, NUM_FORCES, NUM_R);
    normal_error = zeros(1000, NUM_FORCES, NUM_R);
    normal_FBerror = zeros(1000, NUM_FORCES, NUM_R);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Main loop: passes problems to solvers and records stats
    fprintf(['Running ' num2str(NUM_SOLVERS) ' solvers on ' num2str(NUM_PROBLEMS)  ' problems...']);
    fprintf('       %% complete \n');
    progress = 0; 
    % For each r
    P = DATA.(PROBNAMES{1});
    
    P.solver.errmetric = struct();
    P.solver.errmetric = ErrorParams;
    for force = 1:NUM_FORCES
        %%%%%%%%%%%%%%%%%%%%%%% Main loop on r %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for  pr  =  1 : NUM_R
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
                
                % Before calling the solver:
                % Change the external forces
                if isfield(P, 'dynamics')
                    P.dynamics.Forces(3, 1) =  External_Forces(force);   
                end
                if isfield(P, 'Ndynamics')
                    P.Ndynamics.Forces(3, 1) = External_Forces(force);
                end     
                % To make it simple, we will use one solver each time.
                % Also this will be wise since different solvers require
                % different r values. So the range might vary a lot
                
                % Solve % we have only one solver
                solution = feval(SOLVER_HANDLES{1}, P);
                
                % Store results of the corresponding solver of a problem
                iterations(force, pr) = solution.iterations;
                solve_time(force, pr) = solution.solve_time;
                total_error(1:solution.iterations, force, pr) = solution.total_error(1:solution.iterations);
                friction_error(1:solution.iterations, force, pr) = solution.friction_error(1:solution.iterations);
                friction_FBerror(1:solution.iterations, force, pr) = solution.fricFBerror(1:solution.iterations);
                normal_error(1:solution.iterations, force, pr) = solution.normal_error(1:solution.iterations);
                normal_FBerror(1:solution.iterations, force, pr) = solution.normFBerror(1:solution.iterations);
                final_absolute_error(force, pr) = abs( solution.total_error(solution.iterations));
                
                percentDone = 100*pr/NUM_R;
                if percentDone > progress+10 && percentDone < 100
                    fprintf([repmat('\b',1,12) num2str(round(percentDone)) '%% complete']);
                    progress = percentDone;
                end
            end
        end % for pr = 1 : NUM_R
    end % for problem = 1 : NUM_PROBLEMS
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
    force_range = 1:NUM_FORCES;  % all the r values
    avg_tot = zeros(NUM_R, 1000);    
    
    for pr = 1 : NUM_R
        tot_err = total_error(1:max(iterations(force_range, pr)), force_range, pr)'; % NUM_Forces by Iterations
        avg_tot(pr, 1:size(tot_err, 2)) = mean(tot_err, 1);
    end
 
    color2use = hsv(NUM_FORCES);
    
    % Make up the string array for the legends
    f_legend = cell(1, NUM_FORCES);
    for k = 1 : NUM_FORCES
        f_legend{k} = strcat('g = ', num2str(External_Forces(k)));
    end 
    
    % -1*[0  5  5+1e-5  10  1e3  1e6  1e9];
    %f_legend = {'g=0'  'g=-5'  'g= -(5+\epsilon)' 'g=-10' 'g= -1e3'  'g= -1e6' 'g= -1e9'};
    f_legend = {'g=0'  'g=-5'  'g= -(5+\epsilon)' 'g=-10' 'g= -1e3' 'g=-1e6'};
    %f_legend = {'10'  '100'  '500' '510' '510+\epsilon' '520'  '530' '550' '1e6' '1e8' '1e9'}; 
   % sn = cell{NUM_R, 1};
    count = 1;  

 
    R_range = 1 : NUM_R;
    r_legend = cell(1, NUM_R);
    for k = 1 : NUM_R
        r_legend{k} = strcat('r = ', num2str(Rvalues(k)));
    end 
    
%      for pf = 1 : NUM_FORCES   
%          figure;
%          total_error_per_force = total_error(1:max(iterations(pf, R_range)), pf, R_range);
%           for j = 1 : NUM_R
%                semilogy(total_error_per_force(:, :, j), '--*',  'Color', color2use(j, :));
%                hold on;
%           end
%          legend(r_legend, 'FontSize', 12);
%          xlabel('Solver Iteration',   'FontSize', 14,  'FontWeight','bold');
%          ylabel('total error',  'FontSize', 14,  'FontWeight','bold');
%          title(strcat('total error for different r values with g= ', num2str(External_Forces(pf))), 'FontSize', 14,  'FontWeight', 'bold');
%          count = count + 1;
%          hold off;
%      end   
%    
figure;
for force = 1 : NUM_FORCES
    plot(Rvalues, iterations(force, :)', '-', 'LineWidth', 2', 'Color', color2use(force, :));
    hold on;
end
legend(f_legend, 'FontSize', 12);
xlabel('Value of convergence parameter r', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('The number of iterations', 'FontSize', 14, 'FontWeight', 'bold');
title('The number of iterations with mass = 10', 'FontSize', 14, 'FontWeight', 'bold');
end
     
    
    
    
    
    
    