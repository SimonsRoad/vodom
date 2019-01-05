function [P,P_cand,P_cand_orig,T_cand_orig,X,discard,discard_cand] = ...
    postProcessing(P, P_cand, P_cand_orig, T_cand_orig, X, ...
    discard, discard_cand, max_discard, max_cand_discard)
% Discard landmarks that are behind the camera as they are surely outliers.
discard(X(3,:) < 0) = inf; 
% Increment non-selected as keypoint candidates counter.
discard_cand = discard_cand + 1; 
% Delete correspondences and candidates with too large counter. 
keep = discard <= max_discard; 
P = P(:, keep); 
X = X(:, keep); 
discard = discard(keep); 
keep = discard_cand <= max_cand_discard; 
P_cand = P_cand(:, keep); 
P_cand_orig = P_cand_orig(:, keep); 
T_cand_orig = T_cand_orig(:, keep); 
discard_cand = discard_cand(keep); 
end