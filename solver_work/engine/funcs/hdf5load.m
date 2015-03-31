
% Ummm, so the data is all there and readable but I'm having trouble
% getting the single struct holding it all when the .h5 file was created in
% Octave.  

% Given a .h5 file, loads the first dataset.  This is useful in cases where
% there is only a single dataset, e.g. a struct that holds everything!

function data = hdf5load( filename )
    info = hdf5info( filename );
    data = hdf5read( filename, ...
                     info.GroupHierarchy.Groups(1).Datasets(1).Name );
end

