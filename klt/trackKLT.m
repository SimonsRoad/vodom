function [delta, keep] = trackKLT(I_prev, I, keypoint, params)
% Determine keypoint shift from previous to current image in image frame
% by applying the (robust) KLT algorithm, i.e. check KLT resulting warping
% by checking bidirectional error of original to inverse warped image. 
% @param[in]    I_prev      reference image.
% @param[in]    I           image to track point in.
% @param[in]    keypoint    point to track, expressed as [x y]=[col row].
% @param[out]   delta       delta by which the keypoint has moved between 
%                           images.
% @param[out]   keep        true if the point tracking has passed the
%                           bidirectional error test.
r_T         = params('patch_size'); 
num_iters   = params('klt_num_iters'); 
lambda      = params('klt_lambda'); 
W = warping(I_prev, I, keypoint, r_T, num_iters);
delta = W(:, end);
Winv = warping(I, I_prev, (keypoint'+delta)', r_T, num_iters);
dkpinv = Winv(:, end);
keep = norm(delta + dkpinv) < lambda;
end

