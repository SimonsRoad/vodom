%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Detect keypoints (feature) using the Harris feature detector.   
% URL: https://www.mathworks.com/help/vision/local-feature-extraction.html
% Describe keypoints using simple square neighbhorhood extraction. 
% URL: https://www.mathworks.com/help/vision/ref/extractfeatures.html
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [S, D] = keypoints(img, num_keypoints, quality, desc_size)
% @param[in]    img             image to detect keypoints in. 
% @param[in]    num_keypoints   maximal number of keypoints to select. 
% @param[in]    quality         feature quality parameter. 
% @param[in]    desc_size      	descriptor block size (centered around
%                               corner pixel). 
% @param[out]   S               set of feature pixel coordinates (N,2).
% @param[out]   D               set of feature descriptors (N,M) as 
%                               feature vector length is M=desc_size^2. 
corners = detectHarrisFeatures(img, 'MinQuality', quality);
S = corners.selectStrongest(num_keypoints).Location; 
[D,S] = extractFeatures(img, S, 'Method','Block', 'BlockSize',desc_size);
S = S'; 
end