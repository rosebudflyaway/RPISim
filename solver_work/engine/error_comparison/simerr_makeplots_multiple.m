%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% simerr_makeplots_multiple.m
%  
% Make plots of error data from multiple simerr runs
% (used with the test_slender_rod() simulation)



function ph = simerr_makeplots_multiple(DIRNAME, SIMNAME)


if (1 == nargin)
    % default to current directory
    SIMNAME = DIRNAME;
    DIRNAME = './';
end



% Get a list of the error data in the specified directory
[FL, formulations, idx_GTD] = get_file_list(DIRNAME, SIMNAME);


%  % Crunch the list of error data to get error values for each simulation
[ED, ED_median] = get_simerror_data(DIRNAME, SIMNAME, FL, formulations, idx_GTD);




%  % Plot all the error data
ph = plot_err(DIRNAME, SIMNAME, ED, ED_median, formulations);



fprintf(1, '******** DONE ********\n');













%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [FL, formulation_list, idx_GTD] = get_file_list(DIRNAME, SIMNAME)
% Scan the specified directory for simulation data matching SIMNAME

% FL - list of all simulations found
% formulation_list - PEG, ST, etc.
% idx_GTD - index of ground truth data for each simulation in FL

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
fprintf(1, '*** Building list of formulations and searching for smallest timestep.\n');
formulation_list = {};
for incr_f = 1:length(FL)
    
    clear sim;
    this_fname = strcat(FL{incr_f}, '/sim.mat');

    if exist(this_fname)
        load(this_fname);
        this_formulation = sim.FORMULATION;
            
    
        % Add this simulation to the formulation list
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
        
        
        
        
        
        
        
        
        % Find the PEG simulation with the smallest timestep to use as the ground truth
        if (strcmp('PEG', this_formulation))
            
            if (0 == smallest_timestep)
                smallest_timestep = sim.h;
            end
            
            if (sim.h < smallest_timestep)
                smallest_timestep = sim.h;
            end
            
        end
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        
    else
        fprintf(1, '   !!! Simulation %s is incomplete (missing sim.mat).\n', FL{incr_f});
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



% idx_GTD needs to be the index of the correct GTD simulation for each simulation in FL
fprintf(1, '*** scanning %d simulations for ground truth data.\n', length(FL));

GTD_list = [];
for incr_f = 1:length(FL)
    
    clear sim;
    this_fname = strcat(FL{incr_f}, '/sim.mat');

    if exist(this_fname)
        load(this_fname);
        this_formulation = sim.FORMULATION;
    
        if (strcmp('PEG', this_formulation) && smallest_timestep == sim.h)
            % This is a ground truth simulation
            
            % get the "signature"
            this_log_fname = strcat(FL{incr_f}, '/sim_log.txt');
            if exist(this_log_fname)
                
                this_sigdata = dlmread(this_log_fname, ',', 'L1..T1');
            
                % GTD signature (increment, initial data)
                this_GTD = [incr_f, this_sigdata];
            
                GTD_list = [GTD_list; this_GTD];
            else
               fprintf(1, '   !!!  Simulation %s is missing sim_log.txt.\n', FL{incr_f});
            end
        end
    end
    
end
if (1 == size(GTD_list,1))
    fprintf(1, ' ---> One ground truth simulation found.\n');
else
    fprintf(1, ' ---> %d ground truth simulations found.\n', size(GTD_list,1));
end

%  disp('GTD_list:');
%  disp(GTD_list);


% go through and compare the initial states
fprintf(1, '*** matching simulations to ground truth simulations.\n');
idx_GTD = zeros(length(FL), 1);
for incr_f = 1:length(FL)
    
    this_idx = find(incr_f == GTD_list(:,1));
    if isempty(this_idx)
        
        % get the "signature"
        this_log_fname = strcat(FL{incr_f}, '/sim_log.txt');
        if exist(this_log_fname)
        
            this_sigdata = dlmread(this_log_fname, ',', 'L1..T1');
        
            for incr_g = 1:size(GTD_list,1)
                gtd_sigdata = GTD_list(incr_g, 2:end);
            
                if (this_sigdata == gtd_sigdata)
                    % found a match
                    idx_GTD(incr_f) = GTD_list(incr_g,1);
                end
            end
            if (0 == idx_GTD(incr_f))
                fprintf(1, '   !!! No matching GTD found for simulation %s.\n', FL{incr_f});
            end
        
        else
            fprintf(1, '   !!!  Simulation %s is missing sim_log.txt.\n', FL{incr_f});
        end
        
    else
        % this is a ground truth simulation
        
        idx_GTD(incr_f) = GTD_list(this_idx,1);        
            
    end
end
fprintf(1, '*** matched %d simulations with ground truth data.\n', length(FL));


%  disp('idx_GTD =');
%  disp(idx_GTD);



%  fprintf(1, '*** Ground truth data is index %d, timestep %5.2g (%s)\n', idx_GTD, smallest_timestep, FL{idx_GTD});











%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ED, ED_median] = get_simerror_data(DIRNAME, SIMNAME, FL, formulations, idx_GTD)
% Crunch the error data from the file list, and compare to the Ground Truth Data

% FL - list of simulations (directory names)
% formulations - list of formulations ('PEG', 'ST', etc.)
% idx_GTD - index of the ground truth data for each simulation


fprintf(1, '==================================================================\n');
fprintf(1, '*** LOADING ERROR DATA FROM FILES\n');
fprintf(1, '==================================================================\n');

% Get data for ground truth simulation




% FIXME:
%  ED_long = zeros(length(FL),3);
%  ED_long(1,:) = [GTD(2,1) - GTD(1,1), 1, 0.0];


% Get the data:
current_GTD_idx = 0;
ED_long = [];

for incr_f = 1:length(FL)
    


    
    
    
    % load simulation parameters
    clear sim;
    sim_fname = strcat(FL{incr_f}, '/sim.mat');
    if exist(sim_fname);
        load(sim_fname);
    
        this_formulation = sim.FORMULATION;
        this_timestep = sim.h;

    

        % Load GTD (if not already loaded)
        current_GTD_idx_tmp = idx_GTD(incr_f);
        if (current_GTD_idx ~= current_GTD_idx_tmp)
            current_GTD_idx = current_GTD_idx_tmp;
    
            this_GTD = load_simdata(FL{current_GTD_idx});
    
        end
    
    
    
        % find formulation index
        for incr_fp = 1:length(formulations)
            if (strcmp(this_formulation, formulations{incr_fp}))
                this_formulation_idx = incr_fp;
            end
        end
    

    
    
    
        % load simulation state data from file
        this_sdata = load_simdata(FL{incr_f});



        % Create plots of the trajectory, energy, and etc
        ph_sim = plot_sim(DIRNAME, SIMNAME, FL{incr_f}, this_sdata, sim);
    
    
        % Calculate the error for this simulation run
        this_sim_error = calc_simerror(this_GTD, this_sdata);

        
        
        
        if isempty(ED_long)
            % make the first entry
            
            ED_long(1).formulation = this_formulation;
            ED_long(1).formulation_idx = this_formulation_idx;
            ED_long(1).timestep = this_timestep;
            ED_long(1).sim_error = this_sim_error;
            
        
        else
                
            % find if this formulation and timestep has been done before
            ED_idx = 0;
            for incr_ed = 1:length(ED_long)
                if (ED_long(incr_ed).formulation_idx == this_formulation_idx) && (ED_long(incr_ed).timestep == this_timestep)
                    ED_idx = incr_ed;
                end
            end
            
            
            
            if (0 == ED_idx)
                % it has not (add a new one)
                ED_idx = length(ED_long) + 1;
                
                ED_long(ED_idx).formulation = this_formulation;
                ED_long(ED_idx).formulation_idx = this_formulation_idx;
                ED_long(ED_idx).timestep = this_timestep;
                ED_long(ED_idx).sim_error = this_sim_error;
            
            else
                % it has (append the error data to the existing one
                
                ED_long(ED_idx).sim_error = [ED_long(ED_idx).sim_error; this_sim_error];
                
            end
            
            
            
        
        end % (else) isempty(ED_long)
        
    end % if exist(sim_fname)
    
end % for incr_f = 1:length(FL)
    

fprintf(1, '*** Packed error data into %d different simulations.\n', length(ED_long));
    
    



fprintf(1, '*** Calculating mean error for each simulation.\n');

for incr_e = 1:length(ED_long)
    ED_long(incr_e).sim_error_mean = mean(ED_long(incr_e).sim_error);
    ED_long(incr_e).sim_error_max = max(ED_long(incr_e).sim_error);
    ED_long(incr_e).sim_error_median = median(ED_long(incr_e).sim_error);

    fprintf(1, '   ---> [%d - %s %4.3g] mean = %4.2g, median = %4.2g, max = %4.2g\n', incr_e, ED_long(incr_e).formulation, ED_long(incr_e).timestep, ED_long(incr_e).sim_error_mean, ED_long(incr_e).sim_error_median, ED_long(incr_e).sim_error_max);
end






% Go through ED_long and build ED
% ED = [timestep, PEG, ST, AP]


fprintf(1, '*** Packing error data for plotting.\n');
ED_timesteps_all = zeros(length(ED_long),1);
for incr_e = 1:length(ED_long)
    ED_timesteps_all(incr_e) = ED_long(incr_e).timestep;
end
ED_timesteps = unique(ED_timesteps_all);

ED = zeros(length(ED_timesteps), 1 + length(formulations));
ED(:,1) = ED_timesteps;

ED_median = zeros(length(ED_timesteps), 1 + length(formulations));
ED_median(:,1) = ED_timesteps;


for incr_e = 1:length(ED_long)
    idx_timestep = find(ED_long(incr_e).timestep == ED_timesteps);
    idx_formulation = 1 + ED_long(incr_e).formulation_idx;

    fprintf(1, '     ---> [%d] Found timestep %d, and formulation %d.\n', incr_e, idx_timestep, idx_formulation - 1);

    ED(idx_timestep, idx_formulation) = ED_long(incr_e).sim_error_mean;
    ED_median(idx_timestep, idx_formulation) = ED_long(incr_e).sim_error_median;
end


disp('ED =');
disp(ED);

disp('ED_median =');
disp(ED_median);






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sdata = load_simdata(simname)


fname = strcat(simname, '/sim_log.txt');

fprintf('*** Reading data from %s\n', fname);
sdata_all = csvread(fname);
sdata = sdata_all(:,1:8); % First body only

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
            fprintf(1, '      - calc_simerror(): maximum iterations reached trying to find %5.4g s!\n', sdata(incr_s,1));
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



%  s_err = mean(err_list);
%  fprintf(1, '* mean error: %4.3g\n', s_err);


s_err = err_list;





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [KE, PE] = calc_energy(sdata, h, grav, mass, J)
%% Calculate the kinetic and potential energy for each timestep of a simulation


% initialize
KE = zeros(size(sdata,1), 1);
PE = KE;


% first order velocity estimate
for incr_i = 2:size(sdata,1)
    
    % estimate translational velocity (first order)
    v_trans = norm([sdata(incr_i,2) - sdata(incr_i-1,2); ...
                    sdata(incr_i,3) - sdata(incr_i-1,3); ...
                    sdata(incr_i,4) - sdata(incr_i-1,4)]) / h;
    
    ke_trans = (mass * v_trans^2) / 2;


    % estimate rotational velocity (first order)
    q1 = sdata(incr_i,5:8);
    q0 = sdata(incr_i-1,5:8);
    
    r = quatmultiply(q1, quatinv(q0));
    
    omega_r = [r(2); r(3); r(4)] / sqrt(r(2)^2 + r(3)^2 + r(4)^2);
    
    ke_rot = (J(1,1) * omega_r(1)^2 + J(2,2) * omega_r(2)^2 + J(3,3) * omega_r(3)^2) / 2;
    
    
    KE(incr_i) = ke_trans + ke_rot;
    
    this_PE = 0;
    for incr_g = 1:3
        this_PE = this_PE + mass * grav(incr_g) * sdata(incr_i,incr_g+1);
    end
    PE(incr_i) = this_PE;
    
    
end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ph = plot_err(DIRNAME, SIMNAME, ED, ED_median, flist)
% Plot the error data




%  color_list = [0.0, 0.0, 0.7; 0.0, 0.7, 0.0; 0.7, 0.0, 0.7];
color_list = [0.0, 0.7, 0.0; 0.7, 0.0, 0.7; 0.0, 0.0, 0.7];


%%%%%%%%%%%%%%
% Plot mean: %
%%%%%%%%%%%%%%

fh = figure();
set(fh, 'Color', [1 1 1]);
hold on;

ph = zeros(size(ED,2)-1,1);


for incr_e = 2:size(ED,2)

    this_color = color_list(incr_e-1,:);
    ph(incr_e-1) = plot(ED(:,1), ED(:,incr_e), 'LineWidth', 4.0, 'Color', this_color, 'Marker', 'o');
    
end

xlabel('h (s)');
ylabel('mean error (m)');
legend(flist, 'Location', 'SouthEast');

set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');


% Save this plot:
fname = sprintf('%s%s_err_mean', DIRNAME, SIMNAME);
saveas(fh, strcat(fname, '.fig'));
saveas(fh, strcat(fname, '.pdf'));




%%%%%%%%%%%%%%%%
% Plot median: %
%%%%%%%%%%%%%%%%

fh = figure();
set(fh, 'Color', [1 1 1]);
hold on;

ph = zeros(size(ED_median,2)-1,1); % FIXME: overwrites the plot handle (whatever)


for incr_e = 2:size(ED_median,2)
    
    this_color = color_list(incr_e-1,:);
    ph(incr_e-1) = plot(ED_median(:,1), ED_median(:,incr_e), 'LineWidth', 4.0, 'Color', this_color, 'Marker', 'o');

end

xlabel('h (s)');
ylabel('median error (m)');
legend(flist, 'Location', 'SouthEast');

set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');


% Save this plot:
fname = sprintf('%s%s_err_median', DIRNAME, SIMNAME);
saveas(fh, strcat(fname, '.fig'));
saveas(fh, strcat(fname, '.pdf'));





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ph = plot_sim(DIRNAME, SIMNAME, SIMDIRNAME, sdata, sim)
% Plot the individual simulation data

% State plot
%%%%%%%%%%%%
fh = figure('Color', [1 1 1], 'Visible', 'Off');
hold on;

plot(sdata(:,1), sdata(:,2), 'LineWidth', 3.0, 'Color', [0.7 0 0]);
plot(sdata(:,1), sdata(:,3), 'LineWidth', 3.0, 'Color', [0 0.7 0]);
plot(sdata(:,1), sdata(:,4), 'LineWidth', 3.0, 'Color', [0 0 0.7]);

legend('X', 'Y', 'Z');

xlabel('t (s)');
ylabel('trajectory (m)');
set(gca, 'Box', 'On');

fname = sprintf('%s/%s_traj', SIMDIRNAME, SIMNAME);

saveas(fh, strcat(fname, '.fig'));
saveas(fh, strcat(fname, '.pdf'));

close(fh);

fprintf(1, ' --- saved trajectory plot as %s.{fig,pdf}.\n', fname);


% 3D trajectory
%%%%%%%%%%%%%%%
fh = figure('Color', [1 1 1], 'Visible', 'Off');
hold on;

plot3(sdata(:,2), sdata(:,3), sdata(:,4), 'LineWidth', 3.0);
patch([1 -1 -1 1], [4 4 0 0], [0 0 0 0], 'FaceColor', 'b'); 

xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'Box', 'On');
axis equal;

fname = sprintf('%s/%s_traj3D', SIMDIRNAME, SIMNAME);

saveas(fh, strcat(fname, '.fig'));
saveas(fh, strcat(fname, '.pdf'));

close(fh);

fprintf(1, ' --- saved 3D trajectory plot as %s.{fig,pdf}.\n', fname);



% kinetic and potential energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ke, pe] = calc_energy(sdata, sim.h, sim.P{1}.Aext, sim.P{1}.mass, sim.P{1}.J);


fh = figure('Color', [1 1 1], 'Visible', 'Off');
hold on;

plot(sdata(:,1), ke, 'Color', [0.7 0.0 0.0], 'LineWidth', 3.0);
plot(sdata(:,1), pe, 'Color', [0.0 0.7 0.0], 'LineWidth', 3.0);
plot(sdata(:,1), ke + pe, 'Color', [0.0 0.0 0.0], 'LineWidth', 3.0);

xlabel('Time (s)');
ylabel('Energy (J)');

legend('Kinetic', 'Potential', 'Total');

set(gca, 'Box', 'On', 'Xlim', [0, max(sdata(:,1))]);


fname = sprintf('%s/%s_energy', SIMDIRNAME, SIMNAME);

saveas(fh, strcat(fname, '.fig'));
saveas(fh, strcat(fname, '.pdf'));

close(fh);

fprintf(1, ' --- saved energy plot as %s.{fig,pdf}.\n', fname);





ph = 0;

