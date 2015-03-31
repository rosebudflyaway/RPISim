%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_error.m
%
%% Test a particle in simulation with the CDA and S-T method, produce plots of
%% the error, with the states reset at each timestep
%
% Generates data for simulations over a range of time step values

% FIXME: split this off, and write test_hills_NI_gendata() too


function test_error()



wbh = waitbar(0, 'NOW YOU WAIT', 'Name', 'Simulating System', 'Color', [0.7 0.7 1.0]);







%% Run the simulation with a small timestep to find the 'true' trajectory
true_h = 1e-5;

[B_true, V_true] = test_CDA_sawtooth_init(true_h);
V_true = init_logging(V_true);



%  pBounds = [min([B_true.x]), max([B_true.x]), min([B_true.y]) - 2, max([B_true.y]) + 2];
pBounds = [0, 2, 0, 1.5];

%  %% run ground truth simulation
%  V_true = run_sim(B_true, V_true, V_true, @test_CDA_dynamics, wbh, 0, pBounds);



%  max_T = (5 / 327) * (sqrt(718) - 8); % First impact
max_T = 0.338; % Approximate time to leave the first ramp



% Temporary stuff to run analytical simulation: %%%%%%%%%%%%%%%%%%%%%%%

%  X0 = V_true.x;
%  Y0 = V_true.y;
%  nu0 = V_true.nu;
%  % FIXME: analytical solution only up to the first impact
%  for T = 0.0:0.001:max_T
%      V_true.T = T;
%  
%      V_true = test_ANALYTICAL_dynamics_sawtooth(B_true, V_true, X0, Y0, nu0);
%  
%  
%      % Log position and velocity
%      V_true.xlog = [V_true.xlog; V_true.x];
%      V_true.ylog = [V_true.ylog; V_true.y];
%      V_true.nulog = [V_true.nulog, V_true.nu];
%  
%      V_true.Tlog = [V_true.Tlog; V_true.T];
%  
%  
%  end
%  
%  
%  
%  figure(1);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  hold on;
%  
%  incr_b = 1;
%  %  for incr_b = 1:length(B_true)
%      ph_b = plot(B_true(incr_b).x, B_true(incr_b).y, 'linewidth', 4, 'color', [0 0 0]);
%  %  end
%  
%  ph = plot(V_true.xlog, V_true.ylog, 'LineWidth', 4.0, 'Color', [0.5 0.0 0.0]);
%  
%  xlabel('X');
%  ylabel('Y');
%  set(gca, 'Box', 'On');
%  axis equal;

%  delete(wbh);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We're done for now
%  %  return

    
%%%%%%%%%%%%


h_current = true_h * ones(3,1);
errcond_sim = ones(3,1); % keep make step size larger as error condition is met
incr_h = zeros(3,1);

err_CDA = [];
err_ST = [];
err_AP = [];




num_tests = 0;
%  max_num_tests = 3;
max_num_tests = 15;



while any(errcond_sim)

    %% double the current timestep
    h_current = 2 .* errcond_sim .* h_current;
    

    %% run simulations
    
    
    %% CDA
    if (h_current(1) > 0)
        [B_CDA, V_CDA] = test_CDA_sawtooth_init(h_current(1));
        V_CDA = init_logging(V_CDA);
    
        incr_h(1) = incr_h(1) + 1;
        V_CDA = run_sim(B_CDA, V_CDA, B_true, V_true, @test_CDA_dynamics, wbh, incr_h(1), pBounds, max_T, max_num_tests, 1);
    
        err_CDA = [err_CDA; [V_CDA.h, median(V_CDA.error), max(V_CDA.error), mean(V_CDA.error), sum(V_CDA.error), V_CDA.T]];
    
    else
        disp('CDA method simulation is done');
    end

    %% Stewart-Trinkle
    if (h_current(2) > 0)
        [B_ST, V_ST] = test_CDA_sawtooth_init(h_current(2));
        V_ST = init_logging(V_ST);
    
        incr_h(2) = incr_h(2) + 1;
        V_ST = run_sim(B_ST, V_ST, B_true, V_true, @test_ST_dynamics, wbh, incr_h(2), pBounds, max_T, max_num_tests, 2);
    
        err_ST = [err_ST; [V_ST.h, median(V_ST.error), max(V_ST.error), mean(V_CDA.error), sum(V_ST.error), V_ST.T]];
    
    else
        disp('Stewart-Trinkle method simulation is done');
    end

    %% Anitescu-Potra
    if (h_current(3) > 0)
        [B_AP, V_AP] = test_CDA_sawtooth_init(h_current(3));
        V_AP = init_logging(V_AP);
    
        incr_h(3) = incr_h(3) + 1;
        V_AP = run_sim(B_AP, V_AP, B_true, V_true, @test_AP_dynamics, wbh, incr_h(3), pBounds, max_T, max_num_tests, 3);
    
        err_AP = [err_AP; [V_AP.h, median(V_AP.error), max(V_AP.error), mean(V_AP.error), sum(V_AP.error), V_AP.T]];
    
    else
        disp('Anitescu-Potra method simulation is done');
    end

    
    
    num_tests = num_tests + 1;
    if (num_tests > max_num_tests)
        errcond_sim = zeros(3,1);
    end
    
    % FIXME: set a threshold for error to set errcond_sim
    
%     if (h_current > 1e-2)
%         errcond_sim = zeros(3,1);
%     end
  
  
  
  
end % while all(errcond_sim)
  
delete(wbh);

%  test_CDA_make_plots(B, B_ST, Vd, Vd_ST);


% Plot the trajectories
%  figure(1);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  test_CDA_edges_plot_multi(B, Vd, 2, 1, 'CDA method');
%  test_CDA_edges_plot_multi(B_ST, Vd_ST, 2, 2, 'S-T method');


%% Plot errors



figure(1);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP(:,1), err_AP(:,2), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST(:,1), err_ST(:,2), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA(:,1), err_CDA(:,2), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('median error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');


%  disp('       h         err');
%  for incr_i = 1:size(err_CDA,1)
%      disp(sprintf('CDA: %8.4e  %8.4e', err_CDA(incr_i,1), err_CDA(incr_i,2)));
%      disp(sprintf('S-T: %8.4e  %8.4e', err_ST(incr_i,1), err_ST(incr_i,2)));
%      disp(sprintf('A-P: %8.4e  %8.4e', err_AP(incr_i,1), err_AP(incr_i,2)));
%  end





figure(2);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;


ph_AP = plot(err_AP(:,1), err_AP(:,3), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST(:,1), err_ST(:,3), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA(:,1), err_CDA(:,3), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('maximum error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');


%  disp('       h         err');
%  for incr_i = 1:size(err_CDA,1)
%      disp(sprintf('CDA: %8.4e  %8.4e', err_CDA(incr_i,1), err_CDA(incr_i,3)));
%      disp(sprintf('S-T: %8.4e  %8.4e', err_ST(incr_i,1), err_ST(incr_i,3)));
%      disp(sprintf('A-P: %8.4e  %8.4e', err_AP(incr_i,1), err_AP(incr_i,3)));
%  end





figure(3);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP(:,1), err_AP(:,4), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST(:,1), err_ST(:,4), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA(:,1), err_CDA(:,4), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');


xlabel('h (s)');
ylabel('mean error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');



figure(4);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

ph_AP = plot(err_AP(:,1), err_AP(:,5), 'LineWidth', 4.0, 'Color', [0.0 0.0 0.7], 'Marker', 'o');
ph_ST = plot(err_ST(:,1), err_ST(:,5), 'LineWidth', 4.0, 'Color', [0.7 0.0 0.7], 'Marker', 'o');
ph_CDA = plot(err_CDA(:,1), err_CDA(:,5), 'LineWidth', 4.0, 'Color', [0.0 0.7 0.0], 'Marker', 'o');

xlabel('h (s)');
ylabel('accumulated error (m)');
legend('A-P', 'S-T', 'CDA');
%  set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log', 'YDir', 'reverse');
set(gca, 'Box', 'On', 'Xscale', 'log', 'Yscale', 'log');




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTI %
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Vout = update_V(B, Vin, ld)

Vout = Vin;

Vout.t_solver = [Vin.t_solver; ld.t_solver];
Vout.MCPsize = [Vin.MCPsize; ld.MCPsize];

% Update states
Vout.x = Vin.x + Vin.nu(1) * Vin.h;
Vout.y = Vin.y + Vin.nu(2) * Vin.h;

% Log position and velocity
Vout.xlog = [Vin.xlog; Vin.x];
Vout.ylog = [Vin.ylog; Vin.y];
Vout.nulog = [Vin.nulog, Vin.nu];

% Log penetration depth
this_psi = test_CDA_log_penetration(B, Vout);
if (this_psi < 0.0)
  Vout.psi_list = [Vin.psi_list; this_psi];
end

Vout.Tlog = [Vin.Tlog; Vin.T];
Vout.T = Vin.T + Vin.h;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bv = inBounds(pos, pB)

% pB is [x_min, x_max, y_min, y_max]

bv = true;

% x > pB.x_max
if (pos(1) > pB(2))
    bv = false;
end

% x < pB.x_min
if (pos(1) < pB(1))
    bv = false;
end

% y > pB.y_max
if (pos(2) > pB(4))
    bv = false;
end

% y < pB.y_min
if (pos(2) < pB(3))
    bv = false;
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function err = calc_error(Vt, V)
% Calculate error metric


% FIXME: need a better error metric


pos_err = norm([Vt.x; Vt.y] - [V.x; V.y]);
vel_err = norm(Vt.nu - V.nu);

err = pos_err;



%  VT_idx = find(abs(V.T - Vt.Tlog) < Vt.h);
%  
%  
%  if (isempty(VT_idx))
%      warning('No matching data point found for error metric calculation');
%      err = 0.0;
%  else
%      VT_idx = VT_idx(1);
%  
%      
%      pos_err = norm([Vt.xlog(VT_idx); Vt.ylog(VT_idx)] - [V.x; V.y]);
%  
%      vel_err = norm(Vt.nulog(:,VT_idx) - V.nulog(:,end));
%  
%      %err = (0.9 * pos_err + 0.1 * vel_err);
%      err = pos_err;
%  end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Vout = run_sim(B, Vin, Bt, Vt, dynamics_function, wbh, incr_h, pBounds, max_T, max_num_tests, testnum)
% Run simulation

step_count = 0;
Vin.error = [];


X0 = Vt.x;
Y0 = Vt.y;
nu0 = Vt.nu;


%  %  max_T = 3.0; % maximum simulation time

if (0 == incr_h)
    Vt.T = max_T;
end

while Vin.T <= max_T
    
    Vt.T = Vin.T;
    Vt = test_ANALYTICAL_dynamics_sawtooth(Bt, Vt, X0, Y0, nu0);

%      disp(sprintf('Analytical: T = %6.4f [X; Y] = [%5.2f; %5.2f], nu = [%5.2f; %5.2f]', Vt.T, Vt.x, Vt.y, Vt.nu(1), Vt.nu(2)));

    
%      if (incr_h > 0)
%          % set V to states from V_true
%          % find states in Vt for this timestep
%          VT_idx = find(abs(Vin.T - Vt.Tlog) < Vt.h);
%      
%          if (isempty(VT_idx))
%              warning('Simulation step not found');
%              disp(sprintf('Current T = %6.5f', Vin.T));
%              disp('Tlog =');
%              disp(Vt.Tlog);
%              Vout = Vin;
%              return;
%          end

            
    
        % set trajectory at this timestep to trajectory from Vt
%          VT_idx = VT_idx(1);
%  
%          Vin.x = Vt.xlog(VT_idx);
%          Vin.y = Vt.ylog(VT_idx);
%  
%          Vin.nu = Vt.nulog(:,VT_idx);
    
%      end
    
    
    %% Set all states to analytical solution 
%      Vin.x = Vt.x;
%      Vin.y = Vt.y;
%      Vin.nu = Vt.nu;
    
    if (inBounds([Vin.x; Vin.y], pBounds))
        
        path_failure = 0;
        
        try
            [Vin.nu, log_data] = feval(dynamics_function, B, Vin);
            Vin = update_V(B, Vin, log_data);
        catch
            path_failure = 1;
        end
        
    else
        % out of bounds, that's it
        warning(sprintf('Particle out of bounds at T = %6.4f', Vin.T));
        disp(sprintf('[X; Y] = [%5.2f; %5.2f], nu = [%5.2f; %5.2f]', Vin.x, Vin.y, Vin.nu(1), Vin.nu(2)));
        break;
    end

    if (incr_h > 0)
        % log error data 
        Vin.error = [Vin.error; calc_error(Vt, Vin)];
    end



    % Display current data
    wb_value = (3 * (incr_h - 1) + testnum + Vin.T / max_T) / (3 * max_num_tests);
    if (incr_h > 0)
        wb_str = sprintf('Running simulation %d (%5.2e): [%5.2f] X = %5.2f Y = %5.2f\n', ...
                                        incr_h, ...
                                        Vin.h, ...
                                        Vin.T, ...
                                        Vin.x, ...
                                        Vin.y);
    else
        wb_str = sprintf('Running ground truth simulation: [%5.2f] X = %5.2f\n', ...
                                        Vin.T, ...
                                        Vin.x);
    end
    waitbar(wb_value, wbh, wb_str);

    
    
    
    

    step_count = step_count + 1;

    if (step_count > Vin.max_steps)
        warning('Maximum step count reached');
        break;
    end

    
    if (1 == path_failure)
        warning('Path failed: aborting this simulation');
        break;
    end

end % while V.T <= Vt.T

Vout = Vin;
