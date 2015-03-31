function [h5file] = Sim_hdf5writer(matFile, vargin)
data_set = load(matFile);
name = fieldnames(data_set);
data_set = data_set.(name{1});
if nargin == 2
    Index = vargin;
    dataSize = min(length(fieldnames(data_set)), length(Index) );
else
    dataSize = length(fieldnames(data_set));
    Index = 1:dataSize;
end
info = 'Description about the physical simulation';
directory = matFile(1:(length(matFile)-13));
myhdf5 = strcat(directory, '/SpheresFall.h5');
hdf5write(myhdf5, 'Manifest/toolkit', info);
for iter=1:dataSize
    i=Index(iter);
    CurrentFrame = strcat('frame', num2str(i));
    FrameNameHD = sprintf('mbs_frame%06d', i);
    % bodies, contacts, constraints, settings
    PropertyNames = fieldnames(data_set.(CurrentFrame));
    for j = 1 : length(PropertyNames)
        % might have lowest level variables, check here
        if isempty(fieldnames(data_set.(CurrentFrame).(PropertyNames{j})))
            CurrentDirec = strcat(FrameNameHD, '/', PropertyNames{j});
            details = data_set.(CurrentFrame).(PropertyNames{j});
            % since hdf5 saves the opposite way as matlab, to make sure column major
            % we have to convert matlab to row major
            [m, n] = size(details);
            if m > n
                details = details';
            end
            hdf5write(myhdf5, CurrentDirec, details, 'WriteMode', 'append');
        end
        detailNames = fieldnames(data_set.(CurrentFrame).(PropertyNames{j}));
        % lowest level
        % bodies with velocities, masses....
        % contacts with gap, ids....
        % constraints with jacobian....
        for k = 1 : length(detailNames)
            CurrentDirec = strcat(FrameNameHD, '/', PropertyNames{j}, '/', detailNames{k});
            detailParts = data_set.(CurrentFrame).(PropertyNames{j}).(detailNames{k});
            % make sure we have hdf5 in the column major format:
            [m, n] = size(detailParts);
            if m > n
                detailParts = detailParts';
            end
            hdf5write(myhdf5, CurrentDirec, detailParts, 'WriteMode', 'append');
        end
    end
end
h5file = myhdf5;
end

 