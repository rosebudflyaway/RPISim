% hdf5load.m
%
% datastruct = hdf5load_recursive(datastruct,GroupHierarchy)
%
% Recursive procedure used to walk the Group Hierarchy of a given hinfo
% HDF info struct and load the data in the return struct datastruct.

% Author: Gael Varoquaux <gael.varoquaux@normalesup.org>
% Copyright: Gael Varoquaux
% License: BSD-like

function datastruct = hdf5load_recursion(datastruct,GroupHierarchy)

% Load the datasets (the leafs of the tree):
for i=1:size(GroupHierarchy.Datasets,2)
    data=hdf5read(GroupHierarchy.Datasets(i));
    switch class(data)
	case 'hdf5.h5string'
        data=data.Data;
        continue
	    try
 		if size(data,2)>1
 		    buffer=data ;
 		    data = {} ;
 		    for j=1:size(buffer,2)
 			data{j}=buffer(j).Data;
 		    end
 		else
 		    data=data.Data;
 		end
 	    catch
 	    end
	case 'hdf5.h5array'
	    try
		if size(data,2)>1
		    buffer=data ;
		    data = {} ;
		    for j=1:size(buffer,2)
			data{j}=buffer(j).Data;
		    end
		else
		    data=data.Data;
		end
	    catch
        end
    end
    name=GroupHierarchy.Datasets(i).Name;
    name=strrep(name,'/','.');
    eval(['datastruct' name '= data ;'])
end

% Then load the branches:
% Create structures for the group and pass them on recursively:
for i=[1:size(GroupHierarchy.Groups, 2)]
    name=GroupHierarchy.Groups(i).Name;
    % replace the '/' in the name with '.'
    name=strrep(name,'/','.');
    % evaluate the string inside, which is: for example:
    % datastruct.frame = struct;  to make it a struct 
    eval(['datastruct' name '= struct ;']);
    %eval(['datastruct = datastruct' name ';']);
    datastruct = hdf5load_recursion(datastruct,GroupHierarchy.Groups(i));
end

