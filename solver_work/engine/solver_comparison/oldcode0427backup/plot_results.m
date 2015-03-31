function [] = plot_results()
load('solver_comparison.mat');
 
PATHerr = comparison(:, 2);
FPerr = comparison(:, 4);
FNerr = comparison(:, 6);
 
FPiter = comparison(:, 3);
FNiter = comparison(:, 5);

figure(1)
clf;
set(gca,'FontSize',12);
h1 = semilogy(PATHerr, '-','LineWidth',2,'Color',[0.7 0.1, 0.1]);
grid on;
hold on;
title('PATH solver error message','FontSize',12);
xlabel('Simulation steps','FontSize',12);
ylabel('errors','FontSize',12);
 

figure(2)
clf;
set(gca,'FontSize',12);
h2 = semilogy(FPerr, '-','LineWidth',2,'Color',[0.1 0.7, 0.1]);
grid on; hold on;
title('Fixed point error(NCP)','FontSize',12);
xlabel('Simulation steps','FontSize',12);
ylabel('errors','FontSize',12);
% hold off
% print('-f2','-depsc2','output/FPerror');

figure(3);
clf;
set(gca, 'FontSize', 12);
h3 = semilogy(FNerr, '-', 'LineWidth', 2, 'Color', [0.1, 0.1, 0.7]);
grid on; hold on;
title ('Fischer-Newton error', 'FontSize', 12);
xlabel('Simulation steps', 'FontSize', 12);
ylabel('errors', 'FontSize', 12);
% hold off
% print('-f3', '-depsc2', 'output/FNerror');

figure(4);
clf;
set(gca, 'FontSize', 12);
h4 = plot(FPiter, '-', 'LineWidth', 2, 'Color', [0.1 0.7 0.1]);
grid on; hold on;
title('Iteration number for pixed point', 'FontSize', 12);
xlabel('Simulation steps', 'FontSize', 12);
ylabel('iteration number', 'FontSize', 12);


figure(5);
clf;
set(gca, 'FontSize', 12);
h5 = plot(FNiter, '-', 'LineWidth', 2, 'Color', [0.7 0.1 0.7]);
grid on; hold on;
title('Iteration number for Fisher-Newton', 'FontSize', 12);
xlabel('Simulation steps', 'FontSize', 12);
ylabel('iteration number', 'FontSize', 12);


end