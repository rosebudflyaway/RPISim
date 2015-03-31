%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_ANALYTICAL_dynamics_sawtooth.m
%
% Analytically sovle the dynamics for the sawtooth testcase


function Vout = test_ANALYTICAL_dynamics_sawtooth(B, V, X0, Y0, nu0)

% X0, Y0, nu0 are initial states at the start of ballistic flight

%% !!! These calculations are assuming that
%% !!! X0 = 0.0, Y0 = 0.75, nu0 = [6.0; 1.8], g = 9.81, m = 5.0


% three regimes:
% [0] ballistic flight
% [1] impact
% [2] sliding
% [3] 2nd ballistic flight


% calculate alpha, a and b from the first ramp
a = (B(1).y(2) - B(1).y(1)) / (B(1).x(2) - B(1).x(1));
b = B(1).y(1);

alph = atan(a);


%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIXME: don't repeat calculations of T_i, Xi, Yi, nu_i, T_l

%% calculate time at impact
%  T_i = calc_impact_T(V.grav, X0, Y0, nu0);
T_i = (5 / 327) * (sqrt(718) - 8);

%  %  %  disp(sprintf('Time of impact = %6.4f s', T_i));

%% Calculate states at impact
[Xi, Yi, nu_i] = calc_impact_states(T_i, X0, Y0, nu0, V.grav);
%  disp(sprintf('Impact position (analytical): X = %14.12f, Y = %14.12f', Xi, Yi));
%  
%  
%  Xi = (10 / 109) * (sqrt(718) - 8);
%  Yi = (5 / 109) * (sqrt(718) - 8);
%  disp(sprintf('Impact position (exact): X = %14.12f, Y = %14.12f', Xi, Yi));

% Calculate time that particle leaves ramp
T_l = 1000; % FIXME

%% Calculate states where particle leaves ramp
Xl = B(1).x(2);
Yl = B(1).y(2);
% FIXME: calculate velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




pstate = 0;
if (V.T >= T_i)
    pstate = 2;
end
if (V.T >= T_l)
    pstate = 3;
end




if (0 == pstate)
    % initial ballistic flight
    
    [Xp, Yp, nu_p] = calc_ballistic_states(X0, Y0, nu0, V.T, V.grav);
    
elseif (2 == pstate)
    % sliding
    
    [Xp, Yp, nu_p] = calc_sliding_states(Xi, Yi, nu_i, (V.T - T_i), V.grav, alph, V.mu);

elseif (3 == pstate)
    % 2nd ballistic flight
    
    [Xp, Yp, nu_p] = calc_ballistic_states(Xl, Yl, nu_l, (V.T - T_l), V.grav);
    
end



% output
Vout = V;
Vout.x = Xp;
Vout.y = Yp;
Vout.nu = nu_p;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS


function T = calc_impact_T(g, X0, Y0, nu0)
%% find T when (Xp, Yp) = (Xr, Yr) <--- point of impact on the ramp

%  A = g / 2;
%  B = nu0(2) - a * nu0(1);
%  C = Y0 - a * X0 - b;
%  
%  disp(sprintf('A = %6.4f', A));
%  disp(sprintf('B = %6.4f', B));
%  disp(sprintf('C = %6.4f', C));
%  
%  disp('B^2 - 4 * A * C');
%  disp(B^2 - 4*A*C);
%  
%  T1 = (-B + sqrt(B^2 - 4*A*C)) / (2 * A);
%  T2 = (-B - sqrt(B^2 - 4*A*C)) / (2 * A);
%  
%  T = 0.0;
%  
%  if (T1 < 0)
%      T = T2;
%  end
%  
%  if (T2 < 0)
%      T = T1;
%  end
%  
%  
%  disp(T1);
%  disp(T2);


%%%% From Wolfram Alpha

%  x = X0 + NUx0 * T
% y = Y0 + NUy0*T - g*T^2 / 2
%  y = (BY / BX) * x + BY0

% solve x = X0 + NUx0 * T, y = Y0 + NUy0 * T - (g * T^2) / 2, y = (BY / BX) * x + BY0, for T





% OLD: (solve x = a * t, y = b + c * t - g * t^2 / 2, y = x/2, for x,y,t)

a = nu0(1);
b = Y0;
c = nu0(2);

T = abs((sqrt(a^2 - 4 * a * c + 8 * b * g + 4 * c^2) - a^2 + 2 * a * c) / (2 * g));





%
%
%
%


function [X, Y, nu] = calc_ballistic_states(X0, Y0, nu0, T, g)
%% calculate X and Y for ballistic motion


X = X0 + nu0(1) * T;
Y = Y0 + nu0(2) * T - (g * T^2) / 2;

nu = [nu0(1); nu0(2) - g * T];






%
%
%
%


function [X, Y, nu] = calc_impact_states(T_i, X0, Y0, nu0, g)


[X, Y, nu_i] = calc_ballistic_states(X0, Y0, nu0, T_i, g);

%  nu(1) = nu0(1);
%  nu(2) = nu0(2) - g * T_i;

m = 1/2;

t = [1; m];
n = [-m; 1];

tp = [1; -m];
np = [m; 1];


Pn = 5 * dot(nu_i,n);

Pf = Pn * 0.5; % sticking

% FIXME: check for slipping


Pf = 5 * dot(nu_i, t); % slipping


% velocity (in frame of the ramp)
nu = [dot([Pf / 5; 0], tp); dot([Pf / 5; 0], np)];









%
%
%
%


function [X, Y, nu] = calc_sliding_states(X0, Y0, nu0, T, g, alph, mu)


ax = -(mu * g * cos(alph));
ay = -(mu * g * sin(alph));


X = X0 + nu0(1) * T + (ax * T^2) / 2;
Y = Y0 + nu0(2) * T + (ay * T^2) / 2;

nu = [nu0(1) + ax * T^2; nu0(2) + ay * T^2];





