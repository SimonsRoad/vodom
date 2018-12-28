function [R_CW, t3_CW, best_inlier_mask] = estimateTrafoP3P(P, X, K, t3norm, params)
% Estimate camera pose based on (keypoint, landmark) pairs and intrinsics
% calibration matrix based on RANSAC and P3P algorithm. 
% @param[in]    P           keypoints in image plane [2,N]. 
% @param[in]    X           landmarks in world coordinate frame [3,N]. 
% @param[in]    K           intrinsics matrix [3,3].
% @param[in]    t3norm      translation scaling norm. 
% @param[out]   R_CW        world to camera - rotation [3,3]. 
% @param[out]   t3_CV       world to camera - translation [3,1].

num_iterations   = params('ransac_num_iter');  
pixel_tolerance  = params('ransac_pixel_tolerance');  
k                = params('ransac_num_samples');
min_inlier_count = params('ransac_min_inlier'); 
% Initialize RANSAC.
best_inlier_mask = zeros(1, size(P, 2));
P = flipud(P); % (row, col) to (u, v)
max_num_inliers = 0;
% RANSAC
for i = 1:num_iterations
    % Model from k samples.
    [landmark_sample, idx] = datasample(X, k, 2, 'Replace', false);
    keypoint_sample = P(:, idx);
    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = K\[keypoint_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end
    poses = p3p(landmark_sample, normalized_bearings);
    % Decode p3p output
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * X) + ...
        repmat(t_C_W_guess(:,:,1), [1 size(X, 2)]), K);
    difference = P - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;    
    % If we use p3p, also consider inliers for the alternative solution.
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,2) * X) + ...
        repmat(t_C_W_guess(:,:,2), [1 size(X, 2)]), K);    
    difference = P - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        is_inlier = alternative_is_inlier;
    end
    num_inlier = nnz(is_inlier); 
    if num_inlier > max_num_inliers && num_inlier >= min_inlier_count
        max_num_inliers = num_inlier;        
        best_inlier_mask = is_inlier;
    end
end

if max_num_inliers == 0
    R_CW = [];
    t3_CW = [];
else
    M_C_W = dlt(P(:, best_inlier_mask>0)', X(:, best_inlier_mask>0)', K);
    R_CW = M_C_W(:, 1:3);
    t3_CW = M_C_W(:, end)/t3norm;
end

end

