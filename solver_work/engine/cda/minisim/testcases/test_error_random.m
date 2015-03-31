%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_error_random.m
%
%% Test a particle in simulation with the CDA A-P, and S-T method, with random
%% initial states
%
% Generates data for simulations over a range of time step values



function test_error_random(hours_to_run)


if (0 == nargin)
    hours_to_run = 4;
end


%% Set up main parameters for all simulations

wbh = waitbar(0, 'NOW YOU WAIT', 'Name', 'Simulating System', 'Color', [0.7 0.7 1.0]);


% Run the simulation with a small timestep to find the 'true' trajectory
true_h = 1e-5;
%  true_h = 1e-2;


pBounds = [0, 2, 0, 2];

num_tests = 0;
%  max_num_tests = 4;
max_num_tests = 13;
max_T = 1.5;

sim_DATA.wbh = wbh;
sim_DATA.pBounds = pBounds;
sim_DATA.max_T = max_T;
sim_DATA.max_num_tests = max_num_tests;

%% Read in error data file
try
    load('sim_error_data');
    
    disp('Loaded simulation error data (sim_error_data.mat)');
    disp(sprintf('CDA error (%d items)', size(err_CDA,1)));
    disp(sprintf('S-T error (%d items)', size(err_ST,1)));
    disp(sprintf('A-P error (%d items)', size(err_AP,1)));
catch
    warning('Starting with empty error data');
    err_CDA = [];
    err_ST = [];
    err_AP = [];
end

%% Set functions for initialization and dynamics
init_function = @test_CDA_sawtooth_init;


% run this over and over again for 16 hours!

th = tic;
while (toc(th) < hours_to_run*3600)


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Set up parameters and state data for specific simulation

    [B_true, V_true] = feval(init_function, true_h);

    %% Generate a random position and velocity
    V_true = gen_random_states(V_true);

    %% Save random states for later
    rStates.x = V_true.x;
    rStates.y = V_true.y;
    rStates.nu = V_true.nu;


    %% initialize logging
    V_true = init_logging(V_true);


    %% run ground truth simulation
    V_true = run_sim(B_true, V_true, B_true, V_true, @test_CDA_dynamics, wbh, 0, pBounds, max_T, max_num_tests, 0);





    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    h_current = true_h * ones(3,1);
    errcond_sim = ones(3,1); % keep make step size larger as error condition is met
    incr_h = zeros(3,1);

    %  err_CDA = [];
    %  err_ST = [];
    %  err_AP = [];








    while any(errcond_sim)

        %% double the current timestep
        h_current = 2 .* errcond_sim .* h_current;
    

        %% run simulations
    
    
        %% CDA
        [err_CDA, B_CDA, V_CDA, incr_h(1)] = sim_err(err_CDA, B_true, V_true, h_current, incr_h(1), init_function, @test_CDA_dynamics, rStates, 'CDA', 1, sim_DATA);
    
    
        %% Stewart-Trinkle
        [err_ST, B_ST, V_ST, incr_h(2)] = sim_err(err_ST, B_true, V_true, h_current, incr_h(2), init_function, @test_ST_dynamics, rStates, 'Stewart-Trinkle', 2, sim_DATA);
    
  
        %% Anitescu-Potra
        [err_AP, B_AP, V_AP, incr_h(3)] = sim_err(err_AP, B_true, V_true, h_current, incr_h(3), init_function, @test_AP_dynamics, rStates, 'Anitescu-Potra', 3, sim_DATA);
    
    
    
        num_tests = num_tests + 1;
        if (num_tests > max_num_tests)
            errcond_sim = zeros(3,1);
        end
    
        % FIXME: set a threshold for error to set errcond_sim
    
    %     if (h_current > 1e-2)
    %         errcond_sim = zeros(3,1);
    %     end
  
  
  
  
    end % while all(errcond_sim)
  
    

    %% Save error data
    save('sim_error_data', 'err_CDA', 'err_ST', 'err_AP');

end % the big hours long while

delete(wbh);







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
Vout.error = [];

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


%  pos_err = norm([Vt.x; Vt.y] - [V.x; V.y]);
%  vel_err = norm(Vt.nu - V.nu);
%  
%  err = pos_err;



VT_idx = find(abs(V.T - Vt.Tlog) < Vt.h);


if (isempty(VT_idx))
    warning('No matching data point found for error metric calculation');
    err = 0.0;
else
    VT_idx = VT_idx(1);

    
    pos_err = norm([Vt.xlog(VT_idx); Vt.ylog(VT_idx)] - [V.x; V.y]);

    vel_err = norm(Vt.nulog(:,VT_idx) - V.nulog(:,end));

    %err = (0.9 * pos_err + 0.1 * vel_err);
    err = pos_err;
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Vout = run_sim(B, Vin, Bt, Vt, dynamics_function, wbh, incr_h, pBounds, max_T, max_num_tests, testnum)
% Run simulation

step_count = 0;
Vin.error = [];


X0 = Vt.x;
Y0 = Vt.y;
nu0 = Vt.nu;


% ???
%  if (0 == incr_h)
%      Vt.T = max_T;
%  end

while Vin.T <= max_T
    

    
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
%  
%              
%      
%          % set trajectory at this timestep to trajectory from Vt
%  %          VT_idx = VT_idx(1);
%  %  
%  %          Vin.x = Vt.xlog(VT_idx);
%  %          Vin.y = Vt.ylog(VT_idx);
%  %  
%  %          Vin.nu = Vt.nulog(:,VT_idx);
%      
%      end
    
    
    
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






function V_out = gen_random_states(V_in)

%%%% Bounds %%%%%
Xmin = 0.0;
Xmax = 1.5;

Ymin = 1.0;
Ymax = 1.5;

v_max = 10.0;
%%%%%%%%%%%%%%%%%


V_out = V_in;

V_out.x = rand * (Xmax - Xmin) + Xmin;
V_out.y = rand * (Ymax - Ymin) + Ymin;

V_out.nu = [2 * rand * v_max - v_max; 2 * rand * v_max - v_max];





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function [err_DATA, B, V  incr_h] = sim_err(err_DATA, B_true, V_true, h_current, incr_h, initFh, dynFh, rS, sname, test_num, sD)

% err_DATA - error data (for all simulations)
% B - body data for this simulation
% V - state data for this simulation
% B_true - body data for ground truth simulation
% V_true - state data for ground truth simulation
% h_current - current timestep value
% incr_h - timestep increment
% initFh - initialization function handle
% dynFh - dynamics funciton handle
% rS - random states (x,y,nu)
% sname - simulation name
% test_num - increment of this simulation
% sD - sim_DATA


if (h_current(test_num) > 0)
    [B, V] = feval(initFh, h_current(test_num));

    V.x = rS.x;
    V.y = rS.y;
    V.nu = rS.nu;

    incr_h = incr_h + 1;

    V = init_logging(V);

    V = run_sim(B, V, B_true, V_true, dynFh, sD.wbh, incr_h, sD.pBounds, sD.max_T, sD.max_num_tests, test_num);

%      err_DATA = [err_DATA; [V.h, median(V.error), max(V.error), mean(V.error), sum(V.error)]];
    this_ERR.h = V.h;
    this_ERR.Tlog = V.Tlog;
    this_ERR.t_solver = V.t_solver;
    this_ERR.MCPsize = V.MCPsize;
    this_ERR.error = V.error;
    
    this_ERR.xlog = V.xlog;
    this_ERR.ylog = V.ylog;
    this_ERR.nulog = V.nulog;
    
    this_ERR.err_median = median(V.error);
    this_ERR.err_max = max(V.error);
    this_ERR.err_mean = mean(V.error);
    this_ERR.err_sum = sum(V.error);

    err_DATA = [err_DATA; this_ERR];


else
    disp(str_cat(sname, ' method simulation is already done'));
end

