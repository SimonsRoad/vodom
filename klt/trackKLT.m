function [new_kps, keeps] = trackKLT(I_prev, I, keypoints, params)
% Determine keypoint shift from previous to current image in image frame
% by applying the (robust) KLT algorithm, i.e. check KLT resulting warping
% by checking bidirectional error of original to inverse warped image. 
% @param[in]    I_prev      reference image.
% @param[in]    I           image to track point in.
% @param[in]    keypoint    points to track, expressed as [x y]=[col row].
% @param[out]   new_kps     new keypoint positions. 
% @param[out]   keeps       true if the point tracking has passed the
%                           bidirectional error test.
use_matlab  = params('klt_use_matlab'); 
if use_matlab
    pointTracker = vision.PointTracker;
    initialize(pointTracker, keypoints', I_prev);
    [points,point_validity] = step(pointTracker, I);
    keeps = point_validity'; 
    new_kps = points'; 
else
    r_T         = params('patch_size'); 
    num_iters   = params('klt_num_iters'); 
    lambda      = params('klt_lambda'); 
    new_kps = zeros(size(keypoints)); 
    keeps = true(1, size(keypoints, 2));
    for j = 1:size(keypoints, 2)
        W = warping(I_prev, I, keypoints(:,j)', r_T, num_iters);
        delta = W(:, end);
        Winv = warping(I, I_prev, (keypoint'+delta)', r_T, num_iters);
        dkpinv = Winv(:, end);
        keeps(j) = norm(delta + dkpinv) < lambda;  
        new_kps(:,j) = keypoints(:,j) + delta; 
    end
end
end

