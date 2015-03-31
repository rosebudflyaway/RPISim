%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_calc_error.m
%
% Calculate the mean error for this timestep

function [err_mean, err_max] = test_CDA_calc_error(Va, Vd)


% compare the xlog and ylog data in Va and Vd, at each time T in Vd,
% where Va is the "truth" and Vd is the "erroneous" data


for incr_v = 1:length(Vd.xlog)
  
  % find the index in Va that is the closest to this time T in vd
  
  diff_T = abs(Vd.Tlog(incr_v) - Va.Tlog);
  [Tmin, idx_Va] = min(diff_T);
  

  
  % calculate error
  err_list(incr_v) = norm([Vd.xlog(incr_v) - Va.xlog(idx_Va); ...
                           Vd.ylog(incr_v) - Va.ylog(idx_Va)]);
  
end

err_mean = mean(err_list);
err_max = max(err_list);
