% HDF5loader 
% INPUT: 
% problemFile  --------- The hdf5 .h5 file
% source       ---------        1 --- to use the h5loader 
%                               2 --- to use the HDF5loader & PROCESS & CHANGEINCONSISTENCY
% problem_indicies ----- the problem indices that we are interested 

% OUTPUT: 
% frame_data  ---------- return the frame data .mat file


function frame_data = h5load(h5file, source, problem_indicies)

if source == 1
    temp = hdf5info(h5file);
    x = temp.GroupHierarchy;
    frame_data = struct();
    
    if isempty(problem_indicies)
        dataSize = length(x.Groups);
        Index = 1:dataSize;
    else
        Index = problem_indicies;
        dataSize = min( length(x.Groups), length(Index) );
    end
    
    % all the simulation step
    %for i =  1:length(x.Groups)     % loop over all the simulation steps
    for iter=1:dataSize
        i=Index(iter);
        frame = x.Groups(i);
        frame_name = sprintf('frame%06d', i);
        %frame_name = x.Groups(i).Name(1:end);
        frame_data.(frame_name) = struct();
        
        % Dataset  stepsize on the same level of contacts & constraints
        for n = 1 : length(x.Groups(i).Datasets)
            % Hierarchy name with the absolute path
            DatasetHirName  = x.Groups(i).Datasets(n).Name;
            % pure name to save, here is specially for the step_size
            DatasetPureName = DatasetHirName(length(x.Groups(i).Name)+2 : end);
            if strcmp(DatasetPureName, 'timestep')
                DatasetPureName = 'step_size';
            end
            frame_data.(frame_name).(DatasetPureName) = h5read(x.Filename, DatasetHirName);
        end
        % each step with bodies, constraints and contacts
        for j =  1:length(frame.Groups)  %
            group = frame.Groups(j);
            g_name = group.Name(length(frame.Name)+2:end);
            frame_data.(frame_name).(g_name) = struct();
            % each field as datasest on the lowest level
            for k =  1:length(group.Datasets)
                data = group.Datasets(k);
                d_name = data.Name(length(group.Name)+2 : end);
                f = h5read(x.Filename, data.Name);
                f = f';
                [m, n]  = size(f);
                if m==1 && m < n
                    f = f';
                end
                % contacts: all the PSI are saved as: POSITIVE for no
                % penetration NEGATIVE for penetration
                if (strcmp(d_name, 'depth')) && (f > 0)
                    %the case when penetration depth is saved as positive
                    d_name = 'PSI';
                    frame_data.(frame_name).(g_name).(d_name) = -f;
                else
                    % bodies part: for the special use of logs.h5
                    if strcmp(d_name, 'mass')
                        d_name = 'masses';
                    elseif strcmp(d_name, 'gap')
                        d_name = 'PSI';
                    elseif strcmp(d_name, 'inertia_tensor')
                        d_name = 'inertia';
                    elseif strcmp(d_name, 'inertia_tensors')
                        d_name = 'inertia';
                        % For now, we only have one mu, just save it as
                        % mu_k
                    elseif strcmp(d_name, 'mu_s') || strcmp(d_name, 'mu')
                        d_name = 'mu_k';
                    elseif strcmp(d_name, 'h')
                        d_name = 'step_size';
                    end
                    %            problem_indicies = (5:15);     if strcmp(d_name, 'forces')
                    %                     size(f)
                    %                 end
                    frame_data.(frame_name).(g_name).(d_name) = f;
                end
            end % end for k
        end % end for
    end  % end for iter
    
    
else  % else source = 2, which is data from folder agx
    % HDF5load has loaded all the frames
    DATA = HDF5load(h5file);
    if isempty(problem_indicies)
        % FilterActiveIndicies is to find only the active indicies provided
        % by the user
        DATA = FilterActiveIndicies(DATA);
    else 
        DATA = FilterActiveIndicies(DATA, problem_indicies);
    end
    % TODO: the problem_indices needs to be fixed!!!
    %       it might be empty
    % Change the inconsistent field names  to serve for the dynamics part.
    frame_data = ChangeInconsistency(DATA, problem_indicies);
end

end

