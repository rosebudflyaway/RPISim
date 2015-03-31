%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_simulate_noninteractive.m
%
% Run a noninteractive mini-simulation to compare S-T, CDA, and A-P methods


function test_CDA_simulate_noninteractive(B, V, B_ST, V_ST, B_AP, V_AP)


wbh = waitbar(0, 'NOW YOU WAIT', 'Name', 'Simulating System', 'Color', [0.7 0.7 1.0]);

avg_penetration_depth = 0.0;
max_T = 3.0; % maximum simulation time

% find X limits of bodies for each simulation
Xmin = min([B.x]);
Xmax = max([B.x]);

Xmin_ST = min([B_ST.x]);
Xmax_ST = max([B_ST.x]);

Xmin_AP = min([B_AP.x]);
Xmax_AP = max([B_AP.x]);


V = init_logging(V);
V_ST = init_logging(V_ST);
V_AP = init_logging(V_AP);

dead_count = 0;
dead_count_ST = 0;
dead_count_AP = 0;

step_count = 0;

while min([V.T, V_ST.T, V_AP.T]) <= max_T
    

    if ((V.x >= Xmin) && (V.x <= Xmax)) && dead_count < 100
        [V.nu, log_data] = test_CDA_dynamics(B, V);
        V = update_V(B, V, log_data);
    end


    if ((V_ST.x >= Xmin_ST) && (V_ST.x <= Xmax_ST)) && dead_count_ST < 100
        [V_ST.nu, log_data_ST] = test_ST_dynamics(B_ST, V_ST);
        V_ST = update_V(B_ST, V_ST, log_data_ST);
    end

    if ((V_AP.x >= Xmin_AP) && (V_AP.x <= Xmax_AP)) && dead_count_AP < 100
        [V_AP.nu, log_data_AP] = test_AP_dynamics(B_AP, V_AP);
        V_AP = update_V(B_AP, V_AP, log_data_AP);
    end
    

    % Display current data
    waitbar(min([V.T, V_ST.T, V_AP.T]) / max_T, wbh, sprintf('[[STEP %d]] %5.2f', step_count, min([V.T, V_ST.T, V_AP.T])));

    
    % stop things is the particle is outside for all simulations
    num_out = 0;
    if ((V.x < Xmin) || (V.x > Xmax))
        num_out = num_out + 1;
    end
        
    if ((V_ST.x < Xmin_ST) || (V_ST.x > Xmax_ST))
        num_out = num_out + 1;
    end
    
    if ((V_AP.x < Xmin_AP) || (V_AP.x > Xmax_AP))
        num_out = num_out + 1;
    end
        
    if (3 == num_out)
        warning('All simulations went out of bounds');
        break;
    end
    
    
    
    % If the particle velocity is essentially zero for more than 100 steps,
    % time to end it all
    if (norm(V.nu) <= 1e-3)
        dead_count = dead_count + 1;
    end
    
    if (norm(V_ST.nu) <= 1e-3)
        dead_count_ST = dead_count_ST + 1;
    end
    
    if (norm(V_AP.nu) <= 1e-3)
        dead_count_AP = dead_count_AP + 1;
    end
    
    
    
    
    num_dead = 0;
    if (dead_count > 100)
        num_dead = num_dead + 1;
    end
        
    if (dead_count_ST > 100)
        num_dead = num_dead + 1;
    end
    
    if (dead_count_AP > 100)
        num_dead = num_dead + 1;
    end
    
    
    if (3 == num_dead)
        warning('All simulations went dead');
        break;
    end
    
    
    step_count = step_count + 1;
    
    if (step_count > V.max_steps)
        break;
    end
    
end % min(V.T, V_ST.T, V_AP.T) <= max_T



delete(wbh);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make some plots

%  if (~isempty(psi_list))
%    % Create plot of psi
%    figure(2);
%    set(gcf, 'Color', [1 1 1]);
%    bar(psi_list);
%  else
%    fprintf(1, 'No psi values to plot.\n');
%  end


% Plot the trajectories
figure(1);
clf;
set(gcf, 'Color', [1 1 1]);

test_CDA_edges_plot(B, V, 3, 1, 'CDA method');
test_CDA_edges_plot(B_ST, V_ST, 3, 2, 'S-T method');
test_CDA_edges_plot(B_AP, V_AP, 3, 3, 'A-P method');



% Plot histograms of the solver times
figure(2);
clf;
set(gcf, 'Color', [1 1 1]);

subplot(3,1,1);
bar(V.t_solver);
title('Solver time -- CDA');

subplot(3,1,2);
bar(V_ST.t_solver);
title('Solver time -- S-T');


subplot(3,1,3);
bar(V_AP.t_solver);
title('Solver time -- A-P');


fprintf('Average solver time, CDA: %2.3f ms\n', 1e3 * mean(V.t_solver));
fprintf('Average solver time, S-T: %2.3f ms\n', 1e3 * mean(V_ST.t_solver));
fprintf('Average solver time, A-P: %2.3f ms\n', 1e3 * mean(V_AP.t_solver));


%  figure(3);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  subplot(2,1,1);
%  nu_abs = zeros(size(V.nulog, 2), 1);
%  for incr_nu = 1:size(V.nulog, 2)
%      nu_abs(incr_nu) = norm(V.nulog(:, incr_nu));
%  end
%  plot(V.Tlog, nu_abs, 'Color', [0 0 0], 'LineWidth', 4.0);
%  title('CDA method');
%  xlabel('T (seconds)');
%  ylabel('\nu (m/s)');
%  
%  subplot(2,1,2);
%  nu_abs = zeros(size(V_ST.nulog, 2), 1);
%  for incr_nu = 1:size(V_ST.nulog, 2)
%      nu_abs(incr_nu) = norm(V_ST.nulog(:, incr_nu));
%  end
%  plot(V_ST.Tlog, nu_abs, 'Color', [0 0 0], 'LineWidth', 4.0);
%  title('S-T method');
%  xlabel('T (seconds)');
%  ylabel('\nu (m/s)');



% Plot histograms of the problem size
figure(3);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

%  subplot(2,1,1);
bar(V.MCPsize, 'EdgeColor', [0.0 0.75 0.0], 'FaceColor', [0.0 0.75 0.0]);
%  title('Problem size -- CDA');

%  subplot(2,1,2);
bar(V_ST.MCPsize, 'EdgeColor', [0.75 0.0 0.75], 'FaceColor', [0.75 0.0 0.75]);
%  title('Problem size -- S-T');

bar(V_AP.MCPsize, 'EdgeColor', [0.0 0.0 0.75], 'FaceColor', [0.0 0.0 0.75]);

set(gca, 'Box', 'On');


%  % Plot histograms of the solver time normalized by problem size (NOPE)
%  figure(4);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  subplot(2,1,1);
%  bar(V.t_solver ./ V.MCPsize);
%  title('Specific solving time -- CDA');
%  
%  subplot(2,1,2);
%  bar(V_ST.t_solver ./ V_ST.MCPsize);
%  title('Specific solving time -- S-T');


figure(4);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

test_CDA_edges_drawbodies(B,V);

ph_ST = plot(V_ST.xlog, V_ST.ylog, 'Color', [0.7 0.0 0.7], 'linewidth', 4.0);
ph_CDA = plot(V.xlog, V.ylog, 'Color', [0 0.7 0.0], 'linewidth', 4.0);
ph_AP = plot(V_AP.xlog, V_AP.ylog, 'Color', [0 0.0 0.7], 'linewidth', 4.0);



xlabel('X (meters)');
ylabel('Y (meters)');

set(gca, 'Box', 'On');
legend([ph_CDA, ph_ST, ph_AP], {'CDA', 'S-T', 'A-P'});

axis image;


% Plot energy
%%%%%%%%%%%%%

figure(5);
clf;
set(gcf, 'Color', [1 1 1]);


subplot(3,1,1);
hold on;

title('CDA');
ph_kinetic = area(V.Tlog, V.Klog, 'FaceColor', [0.0 0.7 0.0], 'EdgeColor', [0.0 0.3 0.0], 'linewidth', 1.0);
ph_potential = plot(V.Tlog, V.Ulog, 'Color', [0.0 0.7 0.0], 'linewidth', 2.0, 'LineStyle', '--');
ph_total = plot(V.Tlog, (V.Ulog + V.Klog), 'Color', [0.0 0.7 0.0], 'linewidth', 4.0);

set(gca, 'Box', 'On');
legend([ph_total, ph_potential, ph_kinetic], {'Total', 'Potential', 'Kinetic'});


subplot(3,1,2);
hold on;

title('S-T');
ph_ST_kinetic = area(V_ST.Tlog, V_ST.Klog, 'FaceColor', [0.7 0.0 0.7], 'EdgeColor', [0.3 0.0 0.3], 'linewidth', 1.0);
ph_ST_potential = plot(V_ST.Tlog, V_ST.Ulog, 'Color', [0.7 0.0 0.7], 'linewidth', 2.0, 'LineStyle', '--');
ph_ST_total = plot(V_ST.Tlog, (V_ST.Ulog + V_ST.Klog), 'Color', [0.7 0.0 0.7], 'linewidth', 4.0);

set(gca, 'Box', 'On');
legend([ph_ST_total, ph_ST_potential, ph_ST_kinetic], {'Total', 'Potential', 'Kinetic'});


subplot(3,1,3);
hold on;

title('A-P');
ph_AP_kinetic = area(V_AP.Tlog, V_AP.Klog, 'FaceColor', [0.0 0.0 0.7], 'EdgeColor', [0.0 0.0 0.3], 'linewidth', 1.0);
ph_AP_potential = plot(V_AP.Tlog, V_AP.Ulog, 'Color', [0.0 0.0 0.7], 'linewidth', 2.0, 'LineStyle', '--');
ph_AP_total = plot(V_AP.Tlog, (V_AP.Ulog + V_AP.Klog), 'Color', [0.0 0.0 0.7], 'linewidth', 4.0);

set(gca, 'Box', 'On');
legend([ph_AP_total, ph_AP_potential, ph_AP_kinetic], {'Total', 'Potential', 'Kinetic'});




% Plot velocities
%%%%%%%%%%%%%%%%%

%  figure(6)



%  fprintf(1, 'Final nu = [%5.4f, %5.4f] @ T = %5.4f\n', V.nu(1), V.nu(2), V.T);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Vout = init_logging(Vin)

Vout = Vin;

Vout.psi_list = [];

Vout.t_solver = [];
Vout.MCPsize = [];

Vout.xlog = [];
Vout.ylog = [];
Vout.nulog = [];
Vout.Tlog = [];

Vout.Ulog = [];
Vout.Klog = [];





function Vout = update_V(B, Vin, ld)

Vout = Vin;

Vout.t_solver = [Vin.t_solver; ld.t_solver];
Vout.MCPsize = [Vin.MCPsize; ld.MCPsize];

% Update states
Vout.x = Vin.x + Vin.nu(1) * Vin.h;
Vout.y = Vin.y + Vin.nu(2) * Vin.h;




% Log position and velocity and energy
Vout.xlog = [Vin.xlog; Vin.x];
Vout.ylog = [Vin.ylog; Vin.y];
Vout.nulog = [Vin.nulog, Vin.nu];

this_E = test_CDA_calc_energy(Vin);
Vout.Ulog = [Vin.Ulog; this_E.U];
Vout.Klog = [Vin.Klog; this_E.T];


% Log penetration depth
this_psi = test_CDA_log_penetration(B, Vout);
if (this_psi < 0.0)
    Vout.psi_list = [Vin.psi_list; this_psi];
end

Vout.Tlog = [Vin.Tlog; Vin.T];
Vout.T = Vin.T + Vin.h;


