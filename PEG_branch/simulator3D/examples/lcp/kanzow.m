function [phi, merit] = kanzow(a,b,lambda)
% Compute the Chen-Chen-Kanzow function and its natural merit function, as
% defined % in Kanzow's paper on Penalized Fischer-Burmeister functions.
% Math. Program., Ser. A 88: 211–216 (2000)
%   The CCK function is a convex combination of the  Fischer-Burmeister
% function and the product of max(a,0) and max(b,0).
%
%  usage:  [phi, merit] = kanzow(a,b,lambda)
%
%    a     ->   a matrix of real numbers
%    b     ->   a matrix of real numbers
%  lambda  ->   a real scalar defining the convex combination
%   phi    ->   a matrix containing the Kanzow function values
%  merit   ->   a real scalar, the natural metric of phi
%
%    "a" and "b" must be the same size to compute phi.  The resulting phi
%    will be the same size as "a" and "b".
%    
%    To compute the natural metric, "a" and "b" must both be vectors of the
%    same length.  The vector may be a row or a column.
%
% Related function:
%    kanzow_gradient(a,b,lambda) expects "a" and "b" to be column vectors
%    and returns the gradient as two columns, the first corresponding to
%    the "a" component, the second to the "b" component.

% Written by Jeff Trinkle, Nov 2012.
if nargin < 3
    lambda = 0.95;   % Default value of lambda take from Chen, Chen, Kanzow paper
end
sza = size(a);
szb = size(b);
if sza ~= szb
    error('First two input arguments to kanzow(a,b,lambda) must have same dimensions.');
end

pos_a = max(a,0);
pos_b = max(b,0);
phi = lambda * fischerB(a,b) + (1 - lambda) * pos_a .* pos_b;
if nargout > 1
        if sza(2) == 1
            merit = phi' * phi / 2;   % Merit fcn of vector Kanzow fcn.
        elseif sza(1) == 1
            merit = phi * phi' / 2;   % Merit fcn of vector Kanzow fcn.
        else
            error('Input vectors to Kanzow function, must be same length and shape.');
        end
end

