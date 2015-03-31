%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% simerr_makeplots.m
%  
% Make plots of error data from simerr runs
% (used with the test_slender_rod() simulation)



function ph = simerr_makeplots(DIRNAME, SIMNAME)

ph = 0; % FIXME get rid of this when there are plot handles to send out

if (1 == nargin)
    % default to current directory
    SIMNAME = DIRNAME;
    DIRNAME = './';
end



% Get a list of the error data in the specified directory
[FL, formulations, idx_GTD] = get_file_list(DIRNAME, SIMNAME);



%  % Make a plot of the ground truth data (just to check)



%  % Crunch the list of error data to get error values for each simulation
ED = get_simerror_data(idx_GTD, FL, formulations);




%  % Plot all the error data
plot_err(ED, formulations);














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [FL, formulation_list, idx_GTD] = get_file_list(DIRNAME, SIMNAME)
% Scan the specified directory for simulation data matching SIMNAME

fprintf(1, '*** Building file list\n');

% Get full directory listing
dlist = dir(DIRNAME);

full_SIMNAME = strcat('errdata_', SIMNAME);

% Go through listing and find directories matching SIMNAME
FL = {};
for incr_d = 1:length(dlist)
    if (1 == dlist(incr_d).isdir)
        % it's a directory
        if (strfind(dlist(incr_d).name, full_SIMNAME))
            % add it to the list
            FL = [FL; {dlist(incr_d).name}];
            fprintf(1, '     + adding simulation: %s\n', dlist(incr_d).name);
        end
    end
        
    
end


fprintf(1, '*** %d total simulations found\n', length(FL));


smallest_timestep = 0;
idx_smallest_timestep = 0;

% Build list of formulations used
formulation_list = {};
for incr_f = 1:length(FL)
    
    clear sim;
    load(strcat(FL{incr_f}, '/sim.mat'));
    this_formulation = sim.FORMULATION;

    
    % Find the PEG simulation with the smallest timestep to use as the ground truth
    if (strcmp('PEG', this_formulation))
        
        if (0 == smallest_timestep)
            smallest_timestep = sim.h;
            idx_GTD = incr_f; % index of Ground Truth Data
        end
        
        if (sim.h < smallest_timestep)
            smallest_timestep = sim.h;
            idx_GTD = incr_f; % index of Ground Truth Data
        end
    end
  
    

    if (0 == length(formulation_list))
        formulation_list = {this_formulation};
    else
        formulation_found = 0;
        for incr_l = 1:length(formulation_list)
            if (strfind(formulation_list{incr_l},this_formulation))
                formulation_found = 1;
                break;
            end
        end
        
        if (0 == formulation_found)
            formulation_list = [formulation_list; {this_formulation}];
        end
    end
    
end

if (1 == length(formulation_list))
    fprintf(1, '*** %d formulation found\n', length(formulation_list));
else
    fprintf(1, '*** %d formulations found\n', length(formulation_list));
end

for incr_f = 1:length(formulation_list)
    fprintf(1, '    + found formulation %s\n', formulation_list{incr_f});
end
    
    
fprintf(1, '   + smallest timestep found: %6.3g\n', smallest_timestep);
fprintf(1, '*** Ground truth data is index %d, timestep %5.2g (%s)\n', idx_GTD, smallest_timestep, FL{idx_GTD});





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ED = get_simerror_data(idx_GTD, FL, formulations)
% Crunch the error data from the file list, and compare to the Ground Truth Data


fprintf(1, '*** LOADING ERROR DATA FROM FILES\n');

% Get data for ground truth simulation

fprintf(1, 'Loading ground truth data:\n');
GTD = load_simdata(strcat(FL{idx_GTD}, '/sim_log.txt'));

% Colors for AP, PEG, ST
color_list = [0.0, 0.0, 0.7; 0.0, 0.7, 0.0; 0.7, 0.0, 0.7];

% Make plots of trajectory data %%%%%%%


figure('Color', [1 1 1]);
hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for incr_p = 2:4
    plot(GTD(:,1), GTD(:,incr_p), 'LineWidth', 4.0, 'Color', [0.7, 0.7, 0.7]);
end



ED_long = zeros(length(FL),3);
ED_long(1,:) = [GTD(2,1) - GTD(1,1), 1, 0.0];

% Get the rest of the data
for incr_f = 2:length(FL)
    
    % load simulation parameters
    clear sim;
    load(strcat(FL{incr_f}, '/sim.mat'));
    this_formulation = sim.FORMULATION;

    % find formulation index
    for incr_fp = 1:length(formulations)
        if (strcmp(this_formulation, formulations{incr_fp}))
            this_formulation_idx = incr_fp;
        end
    end
    
    % set color
    this_color = color_list(this_formulation_idx,:);
    fprintf(1, '    - Set color to [%3.2f %3.2f %3.2f] for %s method\n', this_color(1), this_color(2), this_color(3), this_formulation);

    % load simulation state data from file
    this_sdata = load_simdata(strcat(FL{incr_f}, '/sim_log.txt'));

    % add to state plot
    fprintf(1, '   # plotting simulation: %s, timestep %5.2g\n', FL{incr_f}, sim.h);
    for incr_p = 2:4
        plot(this_sdata(:,1), this_sdata(:,incr_p), 'LineWidth', 2.0, 'Color', this_color);
    end
    
    
    
    
    % Calculate the error for this simulation run
    this_sim_error = calc_simerror(GTD, this_sdata);
    ED_long(incr_f,:) = [sim.h, this_formulation_idx, this_sim_error];
    
end



set(gca, 'Box', 'On');
xlabel('time (s)');
ylabel('position (m)');



% Pack the long error data into ED (sort by timestep)
%  ED = zeros(length(FL) / length(formulations), length(formulations) + 1);
ED = [];
fprintf(1, '*** Sorting error data ***\n');
disp('ED_long:');
disp(ED_long);

for incr_e = 1:size(ED_long,1)

    % search through ED, add row if the timestep value isn't already there
    
    % If this is the first record, ad it
    if (isempty(ED))
        
        ED_err = zeros(1, length(formulations));
        ED_err(ED_long(incr_e,2)) = ED_long(incr_e,3);
    
        ED = [ED_long(incr_e,1), ED_err];
            
    end
    
    
    ED_idx = find(ED_long(incr_e, 1) == ED(:,1), 1);
    if (1 == length(ED_idx))
        % found exactly once
        
        ED(ED_idx, ED_long(incr_e,2) + 1) = ED_long(incr_e,3);
        
    else
        % not found, or something's up
        ED_err = zeros(1, length(formulations));
        ED_err(ED_long(incr_e,2)) = ED_long(incr_e,3);
    
        % Add a new row
        ED = [ED; ED_long(incr_e,1), ED_err];
    
    end
    
end

fprintf(1, ' ---> Sorted %d records. ED is %d x %d\n', size(ED_long,1), size(ED,1), size(ED,2));
disp('ED:');
disp(ED);









%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sdata = load_simdata(fname)

fprintf('*** Reading data from %s\n', fname);
sdata_all = csvread(fname);
sdata = sdata_all(:,1:7); % First body only

fprintf(1, '   + %d timesteps read. Final simulation time: %5.4g\n', size(sdata,1), sdata(end,1));

if (sdata(end,1) < 1.75)
    fprintf(1, '    *** WARNING: simulation did not complete ***\n');
end










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s_err = calc_simerror(gtd, sdata)
%% calculate error data (s_err), comparing sdata to gtd
% s_err is scalar



% Check the final time of the simulation
if (sdata(end,1) < gtd(end,1))
    fprintf(1,'*** Simulation is too short (%4.2g vs. %4.2g for ground truth simulation)\n', sdata(end,1), gtd(end,1));
end

err_list = zeros(size(sdata,1),1);
% go through each row of sdata
for incr_s = 1:size(sdata,1)
   
    % find the row with the closest timestep in gtd
    g_iters = 0;

    % initial guess:
    gtd_idx = floor(size(gtd,1) * incr_s / size(sdata,1));
    timestep_length_gtd = gtd(2,1) - gtd(1,1); % assuming fixed timestep here
    % and monotonically increasing!

    while g_iters < 50
        
        if (gtd_idx < 1)
            gtd_idx = 1;
        end
        
        if (gtd_idx > size(gtd,1))
            gtd_idx = size(gtd,1);
        end
        
        err_dist = sdata(incr_s,1) - gtd(gtd_idx,1);
    
        if (abs(err_dist) <= timestep_length_gtd / 2)
            % close enough
            break;
        end
        
        gtd_offset = round(err_dist / timestep_length_gtd);
        gtd_idx = gtd_idx + gtd_offset;
        
        g_iters = g_iters + 1;
        
        if (0 == gtd_offset)
            % close enough
            break;
        end
    
        if (g_iters >= 50)
            fprintf('      - calc_simerror(): maximum iterations reached!\n');
        end
        
    end
    
    
    
    
    if (gtd_idx < 1)
        gtd_idx = 1;
        fprintf(1, '    - Index of ground thruth data was less than 1, using 1.\n');
    end
    
    if (gtd_idx > size(gtd,1))
        fprintf(1, '    - Index of ground thruth data exceeds bounds.\n');
        gtd_idx = size(gtd,1);
    end
    
    % calculate the error metric
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    err_list(incr_s) = norm(sdata(incr_s,2:4) - gtd(gtd_idx,2:4));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
end



s_err = mean(err_list);
fprintf(1, '* mean error: %4.3g\n', s_err);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ph = plot_err(ED, flist)
% Plot the error data

color_list = [0.0, 0.0, 0.7; 0.0, 0.7, 0.0; 0.7, 0.0, 0.7];


figure();
set(gcf, 'Color', [1 1 1]);
hold on;

ph = zeros(size(ED,2)-1,1);


for incr_e = 2:size(ED,2)

    this_color = color_list(incr_e-1,:);
    ph(incr_e-1) = plot(ED(:,1), ED(:,incr_e), 'LineWidth', 4.0, 'Color', this_color, 'Marker', 'o');
    
end

xlabel('h (s)');
ylabel('mean error (m)');
legend(flist);

set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');




