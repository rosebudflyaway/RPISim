function phi = fischerB(a,b)
% Compute the Fischer-Burmeister function.
%
%  usage:  phi = fischerB(a,b)
%
%    a     ->   a matrix of real numbers
%    b     ->   a matrix of real numbers
%   phi    ->   a matrix containing the Fischer-Burmeister function values
%
%    "a" and "b" must be the same size to compute phi.  The resulting phi
%    will be the same size as "a" and "b".    

% Written by Jeff Trinkle, Nov 2012.
phi = a + b - sqrt(a.*a + b.*b);