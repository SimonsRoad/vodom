function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
% @param[in]    p1(3,N): homogeneous coordinates of points in image 0.
% @param[in]    p2(3,N): homogeneous coordinates of points in image 1.
% @param[in]    M1(3,4): projection matrix corresponding to first image.
% @param[in]    M2(3,4): projection matrix corresponding to second image.
% @param[out]   P(4,N):  homogeneous coordinates of 3-D points.
% Sanity checks
[dim,num_points] = size(p1);
[dim2,npoints2] = size(p2);
assert(dim==dim2);
assert(num_points==npoints2);
assert(dim==3);
[rows,cols] = size(M1);
assert(rows==3 && cols==4);
[rows,cols] = size(M2);
assert(rows==3 && cols==4);
P = zeros(4,num_points);
% Linear algorithm
for j=1:num_points
    % Built matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,j))*M1;
    A2 = cross2Matrix(p2(:,j))*M2;
    A = [A1; A2];
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P(:,j) = v(:,4);
end
P = P./repmat(P(4,:),4,1); % Dehomogeneize (P is expressed in homogeneous coordinates)
return


