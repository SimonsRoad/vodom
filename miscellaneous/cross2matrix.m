%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Compute antisymmetric skew matrix from 2D vector (3rd element = 1)
% such that A*y = cross(x,y) for all 3-vectors y.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function A = cross2Matrix(x)
% @param[in]    x       vector (3,1). 
% @param[in]    A       antisymmetric matrix (3,3). 
A = [0    -1  x(2);
     1   0   -x(1);
    -x(2)  x(1)   0];
end
