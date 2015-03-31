%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% simerr_evaluate.m
%  
% Run error tests on multiple simulations,
% generating data for error plots



function simerr_evaluate(tstep_min, SIMNAME, ROFFSET)


if (2 == nargin)
    ROFFSET = 1;
end


% Run simulations multiple times to compare

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set these variables:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Don't actually simulate anything if set to true:
DRY_RUN = false;

% Formulations to compare
sims_to_run = {'PEG', 'ST', 'AP'};




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






if (0 == nargin)
    tstep_min = 1e-3;
    SIMNAME = 'testsim';
end



sm = length(sims_to_run); % number of simulations run at each timestep




%  fh = 1; % print to console
fh = fopen(strcat(SIMNAME, '.log'), 'a');


fprintf(fh, '================================================================\n');
fprintf(fh, '****** NEW TEST RUN STARTED ON %s ******\n', datestr(now, 'YYYY/mm/dd HH:MM'));
fprintf(fh, '================================================================\n');

if (DRY_RUN)
    fprintf(fh, '***** DRY RUN MODE ENABLED: edit simerr_evaluate.m to disable. *****\n');
end

fprintf(fh, '*** Running simulations: ');
for incr_s = 1:length(sims_to_run)
    fprintf(fh, '%s, ', sims_to_run{incr_s});
end
fprintf(fh, '\n');
    

if (fh ~= 1)
    fprintf(1, 'Saving log to %s.log\n', SIMNAME);
end


simtime = 1.75;
%  tstep_avg = 6.1; % with 12 sides on the cylinder
tstep_avg = 1.55; % with 6 sides on the cylinder

fprintf(fh, '*** Initial average timestep duration is %4.2f seconds\n', tstep_avg);


% Generate timestep and timing data:
tstep_max = 0.1;
fprintf(fh, '*** [%s] Using minimum and maximum timesteps of %5.2g - %5.2g\n', SIMNAME, tstep_min, tstep_max);

iters = floor(log(tstep_max / tstep_min) / log(2));
fprintf(fh, '*** Performing %d iterations for %d methods\n', iters + 1, sm);

fprintf(fh, '****** TOTAL ESTIMATED TIME ******\n');
fprintf(fh, '*** Simulation name: %s\n', SIMNAME);
sim_time = estimate_sim_time(fh, simtime, tstep_avg, tstep_min, iters, sm);
fprintf(fh, '**********************************\n');

orig_sim_time = sim_time;
tic_osim = tic;

for incr_i = 0:iters
    
    tstep = tstep_min * 2^incr_i;
    


    sim_time = estimate_sim_time(fh, simtime, tstep_avg, tstep, iters - incr_i, sm);

    
    % run simulations
    t_SIM_e = zeros(sm,1);
    for incr_s = 1:sm

    
        fprintf(fh, '*** [%s] running %s simulation with timestep %5.2g\n', datestr(now, 'YYYY/mm/dd HH:MM'), sims_to_run{incr_s}, tstep);
    
        tic_SIM = tic;
        try
            if (~DRY_RUN)
                test_slender_rod(tstep, sims_to_run{incr_s}, SIMNAME, ROFFSET);
            end
        catch
        fprintf(fh, '**** [%s] ERROR: %s simulation failed!\n', datestr(now, 'YYYY/mm/dd HH:MM'), sims_to_run{incr_s});
        end
        
        t_SIM_e(incr_s) = toc(tic_SIM);
        fprintf(fh, '*** Simulation time for %s: %s\n', sims_to_run{incr_s}, pretty_timestr(t_SIM_e(incr_s)));
        
    end
    
        
    % Calculate new average time step duration
    total_steps = sm * simtime / tstep;
    tstep_avg = sum(t_SIM_e) / total_steps;
    fprintf(fh, '*** setting new average of %2.4f seconds per timestep\n', tstep_avg);

end


real_sim_time = toc(tic_osim);
fprintf(fh, '\n\n**** Test duration: %s\n', pretty_timestr(real_sim_time));

time_error = real_sim_time - orig_sim_time;
if (time_error <= 0)
    time_error_str = 'earlier';
else
    time_error_str = 'later';
end

fprintf(fh, '**** Finished %s %s than original estimate\n', pretty_timestr(abs(time_error)), time_error_str);



fprintf(fh, '================================================================\n');
fprintf(fh, '****** TEST RUN CONCLUDED ON %s ******\n', datestr(now, 'YYYY/mm/dd HH:MM'));
fprintf(fh, '================================================================\n');
fprintf(fh, '\n\n\n');







function te_total = estimate_sim_time(fh, simtime, tstep_avg, tstep_min, iters, sim_methods)


te_f = @(i) simtime * tstep_avg ./ (tstep_min * 2 .^ i);
te_total = sim_methods * sum(te_f(0:iters));


fprintf(fh, '*** Estimated remaining simulation time: %s\n', pretty_timestr(te_total));
fprintf(fh, '*** NOW: [%s]\n', datestr(now, 'YYYY/mm/dd HH:MM'));
fprintf(fh, '*** Estimated time of completion: %s\n', datestr(now+te_total/(3600*24), 'YYYY/mm/dd HH:MM'));




function ts = pretty_timestr(tv)
% ts - time string
% tv - time value in seconds

if (tv < 10)
    ts = sprintf('%4.2f seconds', tv);
end

if (tv < 60) && (tv >= 10)
    ts = sprintf('3.1f seconds', tv);
end

if (tv < 3600) && (tv >= 60)
    num_minutes = floor(tv / 60);
    num_seconds = round(tv - num_minutes * 60);
    ts = sprintf('%d minutes, %d seconds', num_minutes, num_seconds);
end

if (tv < 172800) && (tv >= 3600)
    num_hours = floor(tv / 3600);
    num_minutes = round((tv - num_hours * 3600) / 60);
    ts = sprintf('%d hours, %d minutes', num_hours, num_minutes);
end

if (tv > 172800)
    num_days = round(tv / 86400);
    ts = sprintf('about %d days', num_days);
end



