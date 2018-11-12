%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R. Then, find the correct relative camera pose 
% (among four possible configs) by returning the one that yields points 
% lying in front of the image plane (with positive depth).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R,T] = estimateTransformation(S1,S2,K)
% @param[in]    S1      homogeneous coordinates of point correspondences
%                       in image 1 (2,N).
% @param[in]    S2      homogeneous coordinates of point correspondences 
%                       in image 2 (2,N). 
% @param[in]    K       intrinsics matrix for camera (3,3) (K1=K2). 
% @param[out]   R       correct rotation matrix (3,3). 
% @param[out]   T       correct translation vector (3,1). 
% where [R|t] = T_C1_C0 = T_C1_W is a transformation that maps points
% from the world coordinate system (identical to the coordinate system 
% of camera 0) to camera 1.
% Estimate essential matrix from point correspondences. 
F = estimateFundamentalMatrix(S1,S2);
E = K'*F*K;
% Compute possible rotation matrices and translation vector.
[U,~,V] = svd(E); 
W = [0 -1 0; 1 0 0; 0 0 1]; 
R1 = U*W*V.'; 
R2 = U*W.'*V.'; 
u3 = U(:,end); 
% Ensure rotation matrix are valid, i.e. det(R) = 1. 
Rots(:,:,1) = det(R1)*R1; 
Rots(:,:,2) = det(R2)*R2; 
if norm(u3) ~= 0
    u3 = u3/norm(u3);
end
% Choose correct transformation by taking the one with most points in 
% front of the image plane, determined by triangulation. 
max_num_positives = 0; 
trans_factors = [+1 -1]; 
for k = 1:2
    Rbar = Rots(:,:,k); 
    M0 = K*eye(3,4);
    for i = 1:2
        Tbar = u3*trans_factors(i);
        M1 = K*[Rbar Tbar]; 
        P0 = linearTriangulation(S1,S2,M0,M1);
        P1 = [Rbar Tbar]*P0; 
        num_positives = sum(P0(3,:) > 0) + sum(P1(3,:) > 0);
        if num_positives > max_num_positives
            max_num_positives = num_positives; 
            R = Rbar; 
            T = Tbar; 
        end
    end
end
end
