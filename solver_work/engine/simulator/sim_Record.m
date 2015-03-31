
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_Record.m
%
% Writes simulation data to a directory for playback with sim_Reaplay(). 

function sim_Record( obj )
    if obj.STEP == 1
        mkdir(obj.DATA_DIRECTORY);  % Create directory
        P = obj.P;                  % Record bodies
        save([obj.DATA_DIRECTORY '/bodies.mat'],'P'); 
        % Open file for writing object position and orientation
        obj.DATA_fileID = fopen([obj.DATA_DIRECTORY '/sim_log.txt'],'w');  
    end 
    fprintf(obj.DATA_fileID,'%f,',obj.TIME);
    for i=1:length(obj.P)
        fprintf(obj.DATA_fileID,'%f,%f,%f,%f,%f,%f,%f,',...
            obj.P{i}.u(1),obj.P{i}.u(2),obj.P{i}.u(3),...
            obj.P{i}.quat(1),obj.P{i}.quat(2),obj.P{i}.quat(3),obj.P{i}.quat(4));
    end
    fprintf(obj.DATA_fileID,'\n'); 
end
