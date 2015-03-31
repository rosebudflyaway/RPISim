%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_analytical.m
%
% Test the analytical solution to the sawtooth system

function test_analytical()



% Initial conditions for analytical solution:
%  % Define the particle
%  V.x = 0;  % Initial point
%  V.y = 0.75;
%  V.m = 5;    % mass
%  V.nu = [6.0; 1.8];  % velocity
%  
%  V.mu = 0.5;       % coefficient of friction
%  V.DEFINE_FRICTION = 1; % use friction
%  V.b = 0.0; % vicous force
%  
%  V.grav = 9.81;
%  V.Fext = [0; -V.grav * V.m]; % a little reaction motor or something, plus gravity

[B_true, V_true] = test_CDA_sawtooth_init(0);


V_true.xlog = [];
V_true.ylog = [];
V_true.nulog = [];
V_true.Tlog = [];



max_T = 0.338; % Approximate time to hit the edge of the ramp
%  max_T = (5 / 327) * (sqrt(718) - 8); % Only up to the time of the first impact

%%  Temporary stuff to run analytical simulation: %%%%%%%%%%%%%%%%%%%%%%%




X0 = V_true.x;
Y0 = V_true.y;
nu0 = V_true.nu;


for T = 0.0:0.001:max_T
    V_true.T = T;

    V_true = test_ANALYTICAL_dynamics_sawtooth(B_true, V_true, X0, Y0, nu0);


    % Log position and velocity
    V_true.xlog = [V_true.xlog; V_true.x];
    V_true.ylog = [V_true.ylog; V_true.y];
    V_true.nulog = [V_true.nulog, V_true.nu];

    V_true.Tlog = [V_true.Tlog; V_true.T];


end



figure(1);
clf;
set(gcf, 'Color', [1 1 1]);
hold on;

incr_b = 1;
%  for incr_b = 1:length(B_true)
    ph_b = plot(B_true(incr_b).x, B_true(incr_b).y, 'linewidth', 4, 'color', [0 0 0]);
%  end

ph = plot(V_true.xlog, V_true.ylog, 'LineWidth', 4.0, 'Color', [0.66 0.0 0.0], 'Marker', 'o');

xlabel('X');
ylabel('Y');
set(gca, 'Box', 'On');
axis equal;



