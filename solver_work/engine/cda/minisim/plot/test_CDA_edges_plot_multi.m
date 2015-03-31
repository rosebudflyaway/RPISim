%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_edges_plot.m
%
% Make a plot of trajectories for multiple runs
%
% Based on test_CDA_edges_plot()
 
function test_CDA_edges_plot_multi(B, V, nplots, plot_i, plot_name)
 
 


if (nargin < 3)
  nplots = 1;
plot_i = 1;
plot_name = 'Trajectory';
end

%% Plot the system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%% Set up plot 
if (1 == nplots)
  figure(1); 
clf;
set(gcf, 'Color', [1 1 1]);
title(plot_name);
else
  subplot(nplots, 1, plot_i);
title(plot_name);
end


xlabel('X (meters)');
ylabel('Y (meters)');
hold on;





test_CDA_edges_drawbodies(B, V);


for incr_v = 1:length(V)

  % more intensity as the plots progress
  this_i = incr_v / length(V);
  V_color = [1.0 - 1.0 * this_i, 0.8, 1.0 - 1.0 * this_i];
  
  % plot the entire trajectory for each instance of V
  plot(V(incr_v).xlog, V(incr_v).ylog, 'Color', V_color, 'linewidth', 3.0);

end


set(gca, 'Box', 'On');
axis equal;


