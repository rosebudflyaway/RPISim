
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function test_CDA_make_plots(B, B_ST, Vd, Vd_ST)

% Plot the trajectories
figure(1);
clf;
set(gcf, 'Color', [1 1 1]);

test_CDA_edges_plot_multi(B, Vd, 2, 1, 'CDA method');
test_CDA_edges_plot_multi(B_ST, Vd_ST, 2, 2, 'S-T method');




% Plot the average error in terms of time step
figure(2);
clf;
set(gcf, 'Color', [1 1 1]);
set(gca, 'Box', 'On', 'Xscale', 'log', 'Xgrid', 'on', 'Ygrid', 'on');
hold on;

for incr_h = 1:length(Vd)
    h_data(incr_h) = Vd(incr_h).h;
    h_data_ST(incr_h) = Vd_ST(incr_h).h;

    e_data(incr_h) = Vd(incr_h).avgerr;
    e_data_ST(incr_h) = Vd_ST(incr_h).avgerr;

    em_data(incr_h) = Vd(incr_h).maxerr;
    em_data_ST(incr_h) = Vd_ST(incr_h).maxerr;

end

plot(h_data, e_data, 'Color', [0.7 1 0.7], 'LineWidth', 4.0);
plot(h_data_ST, e_data_ST, 'Color', [0.7 0.7 1], 'LineWidth', 4.0);

plot(h_data, em_data, 'Color', [0 1 0], 'LineWidth', 4.0);
plot(h_data_ST, em_data_ST, 'Color', [0 0 1], 'LineWidth', 4.0);

xlabel('Timestep (seconds)');
ylabel('Error (meters)');

legend('mean error (CDA)', 'mean error (S-T)', 'maximum error (CDA)', 'maximum error (S-T)');



%  % Plot histograms of the solver times
%  figure(2);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  subplot(2,1,1);
%  bar(V.t_solver);
%  title('Solver time -- CDA');
%  
%  subplot(2,1,2);
%  bar(V_ST.t_solver);
%  title('Solver time -- S-T');






%  
%  fprintf('Average solver time, CDA: %2.3f ms\n', 1e3 * mean(V.t_solver));
%  fprintf('Average solver time, S-T: %2.3f ms\n', 1e3 * mean(V_ST.t_solver));




%  figure(3);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  subplot(2,1,1);
%  nu_abs = zeros(size(V.nulog, 2), 1);
%  for incr_nu = 1:size(V.nulog, 2)
%    nu_abs(incr_nu) = norm(V.nulog(:, incr_nu));
%  end
%  plot(V.Tlog, nu_abs, 'Color', [0 0 0], 'LineWidth', 4.0);
%  title('CDA method');
%  xlabel('T (seconds)');
%  ylabel('\nu (m/s)');
%  
%  subplot(2,1,2);
%  nu_abs = zeros(size(V_ST.nulog, 2), 1);
%  for incr_nu = 1:size(V_ST.nulog, 2)
%    nu_abs(incr_nu) = norm(V_ST.nulog(:, incr_nu));
%  end
%  plot(V_ST.Tlog, nu_abs, 'Color', [0 0 0], 'LineWidth', 4.0);
%  title('S-T method');
%  xlabel('T (seconds)');
%  ylabel('\nu (m/s)');
%  
%  
%  
%  % Plot histograms of the problem size
%  figure(4);
%  clf;
%  set(gcf, 'Color', [1 1 1]);
%  
%  subplot(2,1,1);
%  bar(V.MCPsize);
%  title('Problem size -- CDA');
%  
%  subplot(2,1,2);
%  bar(V_ST.MCPsize);
%  title('Problem size -- S-T');



