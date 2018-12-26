function [keypoints, descriptors] = features(img, params, is_contop) 
% Find and describe harris features in input image. 
% @param[in]  img           input image. 
% @param[out] keypoints     keypoint pixel positions [2,N]. 
% @param[out] descriptors   keypoint descriptors [M,N].
% Find Harris features in 3 steps, first determine Harris scores, then
% select maximal score keypoints (and apply maximum suppression so that
% not all keypoints are in the same spot) and at last describe them 
% using sample patch descriptors. 
patch_size      = params('patch_size'); 
kappa           = params('harris_kappa'); 
r_suppression   = params('r_suppression'); 
r_desciptor     = params('r_desciptor'); 
if is_contop
    num_kp      = params('num_keypoints_cont'); 
else
    num_kp      = params('num_keypoints_init'); 
end
sobel_para = [-1 0 1];
sobel_orth = [1 2 1];
Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);
patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');
scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace
scores(scores<0) = 0;
scores = padarray(scores, [1+pr 1+pr]);
% Select keypoints. 
keypoints = zeros(2, num_kp);
r = r_suppression; 
temp_scores = padarray(scores, [r r]);
for i = 1:num_kp
    [~, kp] = max(temp_scores(:));
    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [row;col];
    keypoints(:, i) = kp - r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end
% Describe keypoints. 
r = r_desciptor; 
descriptors = uint8(zeros((2*r+1) ^ 2, num_kp));
padded = padarray(img, [r, r]);
for i = 1:num_kp
    kp = keypoints(:, i) + r;
    descriptors(:,i) = reshape(...
        padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r), [], 1);
end
end