function [body] = initBodies()
% Assume three moving spheres
body(1).radius = 1;
body(1).density = 1;
body(1).mass = 4 / 3 * pi * body(1).radius^3 * body(1).density;
body(1).type = 'sphere';
body(1).mom = 2 / 5 * body(1).mass * body(1).radius^2 * eye(3, 3);
body(1).u = [0; 0; 3; 1; 0; 0; 0];  % [x, y, z, unit quat]
body(1).nu = [0; -1; 0; 0; 0; 0];
%body(1).nu = [0; 0; 0; 0; 0; 0];
body(1).mu = .2;

body(2).radius = 2;
body(2).density = 1;
body(2).mass = 4 / 3 * pi * body(2).radius^3 * body(2).density;
body(2).type = 'sphere';
body(2).mom = 2 / 5 * body(2).mass * body(2).radius^2 * eye(3, 3);
body(2).u = [-9; .2; 5; 1; 0; 0; 0];  % [x, y, z, unit quat]
body(2).nu = [0; .3; 0; 0; .2; 0];
%body(2).nu = [0; 0; 0; 0; 0; 0];
body(2).mu = 1;
end
