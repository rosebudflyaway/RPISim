function active = Aset(x)
% Returns the active set for the given vector x
% The active set is all indices where x_i=0
inds=1:length(x);
inds=inds';
active=inds(x==0);