%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R. Then, find the correct relative camera pose 
% (among four possible configs) by returning the one that yields points 
% lying in front of the image plane (with positive depth).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Rwc,twc] = estimateTransformation(S1,S2,K)
% @param[in]    S1      homogeneous coordinates of point correspondences
%                       in image 1 (2,N).
% @param[in]    S2      homogeneous coordinates of point correspondences 
%                       in image 2 (2,N). 
% @param[in]    K       intrinsics matrix for camera (3,3) (K1=K2). 
% @param[out]   Rwc     correct rotation matrix (3,3). 
% @param[out]   twc     correct translation vector (3,1). 
% where [R|t] = T_C1_C0 = T_C1_W is a transformation that maps points
% from the world coordinate system (identical to the coordinate system 
% of camera 0) to camera 1.
% Estimate essential matrix from point correspondences. 
[F,inlier_indeces] = estimateFundamentalMatrix(S1',S2');
% Filter out outlier points. 
S1 = S1(:,inlier_indeces); 
S2 = S2(:,inlier_indeces); 
% Determine essential from fundamental matrix and disambiguate 
% transformation from essential matrix. 
E = K'*F*K;
cam = cameraParameters('IntrinsicMatrix',K'); 
[Rcw,tcw] = relativeCameraPose(E,cam,S1',S2'); 
% Got camera to world frame transformation, but we want to have the 
% inverse transformation (transpose = inv. for orthogonal matrix !). 
twc = -Rcw'*tcw';
Rwc = Rcw';
end
