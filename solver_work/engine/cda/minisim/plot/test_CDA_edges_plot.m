%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_edges_plot.m
%
% 2D simulator for testing CDA method: plotting function

function test_CDA_edges_plot(B, V, nplots, plot_i, plot_name)


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






if (isfield(V, 'xlog'))
  % plot the entire trajectory
  
  plot(V.xlog, V.ylog, 'Color', [0.0 0.7 0.0], 'linewidth', 4.0);
  
else
  % draw just a single point
  
  drawvec([V.x; V.y], [V.x + V.nu(1); V.y + V.nu(2)]); % Plot vertex's velocity 
  plot(V.x, V.y, 'bo', 'markerfacecolor', [.2 .2 1], 'markersize',6);  % Plot vertex
  %  axis([-4 4 -3 4]);

end


set(gca, 'Box', 'On');
axis equal;

