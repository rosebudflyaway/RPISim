%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_ST_calc_mineps.m
%
% Calculate the minimum epsilon value for generating the active set of contacts

function min_eps = test_ST_calc_mineps(B)


% the minimum epsilon value is equal to 1/2 the largest edge length



min_eps = 0.0;

for incr_b = 1:length(B)

  % Check the dimensions of B.x and B.y vertex vectors
  if (length(B(incr_b).x) ~= length(B(incr_b).y))
    error('Body vector data length mismatch!\n');
  end

  % find the maximum edge length:
  for incr_i = 2:length(B(incr_b).x)
    this_l = norm([B(incr_b).x(incr_i) - B(incr_b).x(incr_i - 1); ...
                   B(incr_b).y(incr_i) - B(incr_b).y(incr_i - 1)]);
    if (this_l > min_eps)
      min_eps = this_l;
    end
  end
  
  
end
  
