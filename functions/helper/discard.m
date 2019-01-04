function [X, P, status] = discard(X_cand, P_cand, max_distance)
% Discard landmarks (X) and corresponding keypoints. 
% Discard landmarks that are very far away (as they most probably
% are created by triangulation depth errors), behind the camera 
% in an unreliable angle. 
status_close   = X_cand(3,:)<max_distance;  
status_infront = X_cand(3,:)>0; 
status = status_close & status_infront;        
% Use the T_CW of the previous frame to bring back the 
% pointcloud to the world frame. 
X = X_cand(:, status);  
P = P_cand(:, status); 