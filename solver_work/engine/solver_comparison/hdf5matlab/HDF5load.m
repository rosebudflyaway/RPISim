% hdf5load.m
%
% datasets = hdf5load(filename)
%
% Loads all the variables in an hdf5 file and returns a structure
% containing all the datasets present in the root of the hierarchy of the
% file.

% Author: Gael Varoquaux <gael.varoquaux@normalesup.org>
% Copyright: Gael Varoquaux
% License: BSD-like

% Modified that no processing on the string values
function datasets = HDF5load(filename)

hinfo=hdf5info(filename);

datasets=struct;
datasets=hdf5load_recursion(datasets,hinfo.GroupHierarchy);

