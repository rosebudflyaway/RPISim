%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_slender_rod.m
%  
% Test of a slender rod hitting a field of cubes, to test
% the effects of rotation on S-T and PEG
%


function test_slender_rod(TSTEP, FORMULATION, SIMNAME, ROFFSET)


%global i_debug i_debug_fh;
%i_debug = 10; % debug level
%i_debug_fh = 1; % where to send debugging messages



if (0 == nargin)
    TSTEP = 0.01; % simulation time step
    FORMULATION = 'PEG';
    SIMNAME = 'e';
    ROFFSET = 1;
end
    
if (1 == nargin)
    FORMULATION = 'PEG';
    SIMNAME = 'e';
    ROFFSET = 1;
end
    
if (4 == nargin)
    GUI_flag = false;
else
    GUI_flag = true;
end



rod_length = 1.0;
%  rod_radius = 0.015;
rod_radius = 0.2;

%  rod_density = 8.25e3; % kg/m^3 (T 600 stainless steel)
rod_density = 900; % kg/m^3 (ABS/Nylon blend)
rod_volume = rod_length * pi * (rod_radius^2);
rod_mass = rod_density * rod_volume;

cyl_num_verts = 6;

% Objects for scene 
R1 = mesh_cylinder(cyl_num_verts, rod_mass, rod_radius, rod_length);

R1.setStatic(0);
R1.color = [1.0 0.2 0.9];
R1.faceAlpha = 0.8;
R1.setPosition([0; 0; 0.5]);
R1.quat = quat([1; 0; 0], -pi/2);
R1.nu = [0; 5.0; 0.2; -2*pi; 0; 0];
%  R1.nu = [0; 4.9; 0.2; -2*pi; 0; 0];


G = make_ground(0.25, 5, 10, ROFFSET);
    

    
% Pack them up
P = [{R1}; G];
%  P = {R1};


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





% Simulator
sim = Simulation(P, TSTEP); 

sim.formulation(FORMULATION);

sim.RECORD_DATA = true;
sim.DATA_DIRECTORY = strcat('errdata_', SIMNAME, '_', datestr(now, 'yyyymmdd_HHMM'), '_', FORMULATION, '_', num2str(TSTEP, '%0.4g'));

sim.gravityON();

if (GUI_flag)
    sim.enableGUI();
else
    sim.disableGUI();
end
 
 
%  sim.setFriction(false);

% run to 1.75 seconds
sim.MAX_ITERS = 1.75 / TSTEP;

sim.run();



save(strcat(sim.DATA_DIRECTORY, '/sim.mat'), 'sim');



%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%

function G = make_ground(cube_w, num_X, num_Y, r_os)

% cube_w : width of each cube
% num_X : number of cubes in X direction
% num_Y : number of cubes in Y direction
% r_os : offset to start reading random data at

% xC : offset of cube field in X
% yC : offset of cube field in Y
% zC : offset of cube field in Z




% Make a field of cubes at regular intervals, at random orientations

%  cube_w = 0.25; % meters (width of each cube)
cube_s = sqrt(3 * (cube_w^2)); % meters (cube spacing)

% Load the random orientations from file:
%  (any n x 4 matrix of random decimal fractions)
%  (they get reused to compare different methods)
load('random_orientations');
%  fprintf(1, 'Read %d random orientations from file.\n', size(random_orientations,1));


% position offsets:
xC = -(num_X - 1) * cube_s / 2;
yC = 0;
zC = 0;


incr_i = 0;
for p_x = 0:cube_s:(num_X - 1) * cube_s
    for p_y = 0:cube_s:(num_Y - 1) * cube_s
       
        r_axis = random_orientations(incr_i + r_os, 1:3);
        r_axis = r_axis / norm(r_axis);
    
        r_angle = random_orientations(incr_i + r_os, 4) * 2 * pi;
    
        incr_i = incr_i + 1;
    
    
        
        tC = mesh_cube();
        tC.scale(cube_w);
        tC.setStatic(1);
        tC.setPosition([xC + p_x; yC + p_y; zC]);
        tC.quat = quat(r_axis, r_angle);
        
        tC.color = [0.6 0.6 0.7];
        tC.faceAlpha = 1.0;
        
        
        if (1 == incr_i)
            G = {tC};
        else
            G = [G; {tC}];
        end
        
    end 
end
