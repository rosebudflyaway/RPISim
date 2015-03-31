%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_error.m
%
%% Test a particle in simulation with the CDA and S-T method, produce plots of
%% the error, with the states reset at each timestep
%
% Generates data for simulations over a range of time step values






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function make_err_plots(fname)


if (nargin < 1)
    fname = 'sim_error_data';
end



load(fname);



%% Organize the data

disp('Organizing data');

err_AP_sorted_by_h = sort_err_by_h(err_AP);
err_ST_sorted_by_h = sort_err_by_h(err_ST);
err_CDA_sorted_by_h = sort_err_by_h(err_CDA);

err_AP_h = sort_err_h_array(err_AP_sorted_by_h);
err_ST_h = sort_err_h_array(err_ST_sorted_by_h);
err_CDA_h = sort_err_h_array(err_CDA_sorted_by_h);

disp(' ... DONE');





%% Plot error statistics vs h

% data is h, med, max, mean, sum

figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP_h(:,1), err_AP_h(:,2), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST_h(:,1), err_ST_h(:,2), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA_h(:,1), err_CDA_h(:,2), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('median error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');







figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;


ph_AP = plot(err_AP_h(:,1), err_AP_h(:,3), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST_h(:,1), err_ST_h(:,3), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA_h(:,1), err_CDA_h(:,3), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('maximum error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');






figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP_h(:,1), err_AP_h(:,4), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST_h(:,1), err_ST_h(:,4), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA_h(:,1), err_CDA_h(:,4), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');


xlabel('h (s)');
ylabel('mean error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');



figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP_h(:,1), err_AP_h(:,5), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST_h(:,1), err_ST_h(:,5), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA_h(:,1), err_CDA_h(:,5), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('accumulated error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');



%% Generate plot of initial positions
l_CDA = length(err_CDA);
disp(sprintf('%d total simulations run', l_CDA));
xd = zeros(l_CDA,1);
yd = zeros(l_CDA,1);

for incr_i = 1:l_CDA
    xd(incr_i) = err_CDA(incr_i).xlog(1);
    yd(incr_i) = err_CDA(incr_i).ylog(1);
end

figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_init = plot(xd, yd, 'Color', [0 0 0], 'MarkerSize', 8.0, 'Marker', 'o', 'LineStyle', '.');

xlabel('x (m)');
ylabel('y (m)');

axis equal;
set(gca, 'Box', 'On');

% Skip the scatter plot:
return;

%% Scatter plot of error vs. time
disp('Generating scatter plot');
figure();
clf;
set(gcf, 'Color', [1 1 1]);
hold on;


for incr_i = 1:length(err_AP)
   ph_AP = scatter(err_AP(incr_i).Tlog, err_AP(incr_i).error, 6, 'blue', '+');
end

for incr_i = 1:length(err_ST)
    ph_ST = scatter(err_ST(incr_i).Tlog, err_ST(incr_i).error, 6, 'magenta', '+');
end

for incr_i = 1:length(err_CDA)
    ph_CDA = scatter(err_CDA(incr_i).Tlog, err_CDA(incr_i).error, 6, 'green', '+');
end

xlabel('T (s)');
ylabel('Error (m)');
set(gca, 'Box', 'On');

disp(' ... DONE');

%%%%%%%%%%%%%%% FUNCTION TIME %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARTY %%%%%%%%%%%%%%%%%%%%%%%%%%%


function ERR_sorted = sort_err_by_h(this_ERR)

% Put together all the median, mean, max, and accumulated error data
% into bins organized by time step value (h)


ERR_sorted(1).h = this_ERR(1).h;
ERR_sorted(1).error = this_ERR(1).error;

for incr_i = 2:length(this_ERR)
    
    
    
    hlist = [];
    for incr_j = 1:length(ERR_sorted)
        hlist = [hlist; ERR_sorted(incr_j).h];
    end
    
    
%      disp('hlist:');
%      hlist
%      disp('this_ERR:');
%      this_ERR(incr_i).h
    
    
    h_i = find(this_ERR(incr_i).h == hlist, 1, 'first');

    if (h_i > 0)
        % append the error to the existing row
        ERR_sorted(h_i).error = [ERR_sorted(h_i).error; this_ERR(incr_i).error];
    else
        % not found, create a new row
        ERR_sorted_temp.h = this_ERR(incr_i).h;
        ERR_sorted_temp.error = this_ERR(incr_i).error;
    
        ERR_sorted = [ERR_sorted; ERR_sorted_temp];
       
    end
    
end

% Calculate the statistics now that everything is sorted
for incr_i = 1:length(ERR_sorted)
   
    ERR_sorted(incr_i).err_median = median(ERR_sorted(incr_i).error);
    ERR_sorted(incr_i).err_max = max(ERR_sorted(incr_i).error);
    ERR_sorted(incr_i).err_mean = mean(ERR_sorted(incr_i).error);
    ERR_sorted(incr_i).err_sum = sum(ERR_sorted(incr_i).error);
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err_h = sort_err_h_array(err_in);

% err_in is sorted by h, but is a struct
% make err_h a matrix to plot

err_h = [];
for incr_i = 1:length(err_in)
   
    err_TEMP = [err_in(incr_i).h, err_in(incr_i).err_median, err_in(incr_i).err_max, err_in(incr_i).err_mean, err_in(incr_i).err_sum];
    err_h = [err_h; err_TEMP];
    
end


    