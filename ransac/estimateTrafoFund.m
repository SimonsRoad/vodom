function [R, t3, cloud, t3norm] = estimateTrafoFund(kp0, kp1, K, t3norm)
% Estimate transformation between previous and current image using 
% fundamental matrix estimate (estimateFundamentalMatrix - function). 
% @param[in]    qm_keypoints    matched camera 0 keypoints [2,L]. 
% @param[in]    dbm_keypoints   matched camera 1 keypoints [2,L]. 
% @param[in]    K               camera intrinsics.
% @param[out]   R               rotation matrix from 0 to 1. 
% @param[out]   u3              translation vector from 0 to 1. 
% @param[out]   cloud           triangulated point cloud [3,L]. 
p0 = [kp0;ones(1,length(kp0))];
p1 = [kp1;ones(1,length(kp1))];
F = estimateFundamentalMatrix(kp0.', kp1.');
E = transpose(inv(K))*F*inv(K);  %#ok<MINV>
% Given an essential matrix, compute the camera motion, i.e.,  
% R and T such that E ~ T_x R. 
[U,~,V] = svd(E);
u3 = U(:,3); 
W = [0 -1 0; 1 0 0; 0 0 1];
Rots(:,:,1) = U*W*V.';
Rots(:,:,2) = U*W.'*V.';
if det(Rots(:,:,1))<0
    Rots(:,:,1)=-Rots(:,:,1);
end
if det(Rots(:,:,2))<0
    Rots(:,:,2)=-Rots(:,:,2);
end
if norm(u3) ~= 0
    u3 = u3/norm(u3);
end
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
M0 = K * eye(3,4); % Projection matrix of camera 1
total_points_in_front_best = 0;
for iRot = 1:2
    R_C1_C0_test = Rots(:,:,iRot);
    for iSignT = 1:2
        T_C1_C0_test = u3 * (-1)^iSignT;        
        M1 = K * [R_C1_C0_test, T_C1_C0_test];
        P_C0 = linearTriangulation(p0,p1,M0,M1);
        % Project in both cameras.
        P_C1 = [R_C1_C0_test T_C1_C0_test] * P_C0;
        num_points_in_front0 = sum(P_C0(3,:) > 0);
        num_points_in_front1 = sum(P_C1(3,:) > 0);
        total_points_in_front = num_points_in_front0 + num_points_in_front1;
        if (total_points_in_front > total_points_in_front_best)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras.
            R = R_C1_C0_test;
            t3 = T_C1_C0_test;
            total_points_in_front_best = total_points_in_front;
        end
    end
end
% Point cloud with estimated transformation. 
M1 = K * [R, t3];
cloud = linearTriangulation(p0,p1,M0,M1);
cloud = cloud(1:3,:); 
% Renormalize scale of translational vector. 
if isnan(t3norm)
    t3norm = norm(t3); 
else
    t3 = t3/t3norm; 
end
end