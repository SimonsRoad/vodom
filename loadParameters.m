function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map; 
parameters('patch_size') = 9;           % radius of patch to track. 
parameters('harris_kappa') = 0.08;
parameters('num_keypoints_init') = 30; %1000; % #keypoints for initialization. 
parameters('num_keypoints_cont') = 400;  % #keypoints for cont. operation.
parameters('r_suppression') = 8;
parameters('r_desciptor') = 9;
parameters('match_lambda') = 5;
parameters('klt_use_matlab') = true;    % use matlab KLT implementation.
parameters('klt_num_iters') = 50;       % number of warping iterations.
parameters('klt_lambda') = 0.1;         % bidirectional error threshold.
parameters('klt_min_num_kps') = 50;     % min. #kps for KLT tracking. 
parameters('ransac_num_iter') = 1000;   % #ransac optimization iterations.
parameters('ransac_pixel_tolerance') = 50; % acceptable pixel reproj. error.  
parameters('ransac_min_inlier') = 30;   % min. #acceptable ransac inlier.
parameters('ransac_num_samples') = 3;   % sample size for ransac.

if ds == 0
    parameters('bootstrap_frames') = [1 2];     
elseif ds == 1
    parameters('bootstrap_frames') = [1 2]; 
elseif ds == 2
    parameters('bootstrap_frames') = [40 41];%[1 10]; % We need to start here, as we get >40 features after frame 40, but only 20 in the beginning, furthermore a large step ensures smal depth error.
else 
    assert(false)
end