%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% Setup.m
%
% This script initializes the simulator environment.  So just RUN IT!  

%% Attempt to compile mex files (may help increase simulation speed)
% So, there is currently debate on when a .mex file is actually faster than
% just a simplified .m file (due to overhead).  I've had very different 
% results on different hardware (jw), but my current thinking is that for 
% small functions, the .m file should be used. 

cd engine/funcs/mex_files;  
% So it turns out that you can't use the wildcard * with mex :(
% mex all .c files
%mex cross3.c;                    
 

%% Set paths
cd ../../../; 
addpath(genpath('engine'));  % Add every subfolder of 'engine'
addpath('engine');           % Add 'engine' itself
addpath(genpath('scripts')); % Set of example and test scripts
addpath('scripts'); 

disp('Finished setting up RPI-MATLAB-Simulator.');


