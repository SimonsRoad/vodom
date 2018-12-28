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
F = estimateFundamentalMatrix(kp0', kp1');
E = transpose(inv(K))*F*inv(K);  %#ok<MINV>
% Singular value decomposition. 
[U,S,V] = svd(E);
% Set smallest singular value to 0. 
diag = zeros(3,1); 
for i = 1:3
    diag(i) = S(i,i); 
end
[~, i] = min(diag);
S(i,i) = 0;
% Compute possible translational and rotational matrices. 
WT = [0 -1 0; 1 0 0; 0 0 0];
Tras = zeros(3,2); 
Tras(:,1) = matrix2Cross(U*WT*S*U'); 
Tras(:,2) = matrix2Cross(-U*WT*S*U');
WR1 = [0 -1 0; 1 0 0; 0 0 1];
WR2 = [0 1 0; -1 0 0; 0 0 1];
Rots = zeros(3,3,2); 
Rots(:,:,1) = U*WR1*V';
Rots(:,:,2) = U*WR2*V';
if det(Rots(:,:,1))<0
    Rots(:,:,1)=-Rots(:,:,1);
end
if det(Rots(:,:,2))<0
    Rots(:,:,2)=-Rots(:,:,2);
end
% Check which combination results in the most valid points (i.e. 
% points in front of cameras).
M0 = K * eye(3,4); % Projection matrix of camera 1
total_points_in_front_best = 0;
for r = 1:2
    for t = 1:2
        t3c = Tras(:,t); 
        Rc = Rots(:,:,r); 
        % Projection matrix of camera 2. 
        M1 = K * [Rc, t3c];
        % Project in both cameras.
        P_C0 = linearTriangulation(p0,p1,M0,M1);
        P_C1 = [Rc t3c] * P_C0;
        % Keep the rotation that gives the highest number of points
        % in front of both cameras.
        num_points_in_front0 = sum(P_C0(3,:) > 0);
        num_points_in_front1 = sum(P_C1(3,:) > 0);
        total_points_in_front = num_points_in_front0 + num_points_in_front1;
        if (total_points_in_front > total_points_in_front_best)
            R = Rc;
            t3 = t3c;
            total_points_in_front_best = total_points_in_front;
        end        
    end
end
% Renormalize scale of translational vector. 
if isnan(t3norm)
    t3norm = norm(t3); 
else
    t3 = t3/t3norm; 
end
% Point cloud with estimated transformation. 
M1 = K * [R, t3];
cloud = linearTriangulation(p0,p1,M0,M1);
cloud = cloud(1:3,:); 
end