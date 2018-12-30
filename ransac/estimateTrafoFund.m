function [R, t3, cloud, t3norm] = estimateTrafoFund(kp0, kp1, K, t3norm)
% Estimate transformation between previous and current image using 
% fundamental matrix estimate (estimateFundamentalMatrix - function). 
% @param[in]    qm_keypoints    matched camera 0 keypoints [2,L]. 
% @param[in]    dbm_keypoints   matched camera 1 keypoints [2,L]. 
% @param[in]    K               camera intrinsics.
% @param[out]   R               rotation matrix from 0 to 1. 
% @param[out]   u3              translation vector from 0 to 1. 
% @param[out]   cloud           triangulated point cloud [3,L]. 
p0 = [flipud(kp0);ones(1,length(kp0))];
p1 = [flipud(kp1);ones(1,length(kp1))];
F = estimateFundamentalMatrix(fliplr(kp0'), fliplr(kp1'));
E = K'*F*K;
% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations
[R,t3] = disambiguateRelativePose(Rots,u3,p0,p1,K,K);
if isnan(t3norm)
    t3 = t3/norm(t3);   % Because we do svd, this will already have unit length!
    t3norm = norm(t3); 
else
    t3 = t3/t3norm; % BUG: Shouldn't we multiply it with t3norm?
end
% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R, t3];
cloud = linearTriangulation(p0,p1,M1,M2);

end