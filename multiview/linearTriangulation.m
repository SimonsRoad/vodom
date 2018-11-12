%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Estimate 3D world points based on pixel correspondences between first and 
% second image and its intrinsics. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ph = linearTriangulation(p1,p2,M1,M2)
% @param[in]    p1  coordinates of points in image 1 (2,N). 
% @param[in]    p2  coordinates of points in image 2 (2,N).
% @parma[in]    M1  projection matrix corresponding to first image (3,4).
% @param[in]    M2  projection matrix corresponding to second image (3,4)
% @param[out]   Ph  homogeneous coordinates of 3-D points (4,N). 
N = size(p1,2); 
Ph = zeros(4,N); 
% Solve for world point coordinate by SVD. 
for k = 1:N
    p1cross = cross2matrix(p1(:,k)); 
    p2cross = cross2matrix(p2(:,k));
    Q = [p1cross*M1; p2cross*M2]; 
    [~,~,V] = svd(Q,0); 
	Ph(:,k) = V(:,4); 
end
% Dehomogenization. 
Ph = Ph./repmat(Ph(4,:),4,1);
end


