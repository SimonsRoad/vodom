function M = cross2Matrix(x)
% CROSS2MATRIX  Antisymmetric matrix corresponding to a 3D vector
%
% Computes the antisymmetric matrix M corresponding to a 3D vector x such
% that M*y = cross(x,y) for all 3D vectors y.
%
% Input: 
%   - x(3,1) : vector
%
% Output: 
%   - M(3,3) : antisymmetric matrix
%
% See also MATRIX2CROSS

M = [0,-x(3),x(2); x(3),0,-x(1);-x(2),x(1),0];
end