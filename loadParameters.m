function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map; 
parameters('patch_size') = 9;           % radius of patch to track. 
parameters('harris_kappa') = 0.08;
parameters('num_keypoints_init') = 200; % #keypoints for initialization. 
parameters('num_keypoints_cont') = 100; % #keypoints for cont. operation.
parameters('r_suppression') = 8;
parameters('r_desciptor') = 9;
parameters('match_lambda') = 5;
parameters('klt_use_matlab') = true;    % use matlab KLT implementation.
parameters('klt_num_iters') = 50;       % number of warping iterations.
parameters('klt_lambda') = 0.1;         % bidirectional error threshold.

if ds == 0
    parameters('bootstrap_frames') = [1 2];     
elseif ds == 1
    parameters('bootstrap_frames') = [1 2]; 
elseif ds == 2
    parameters('bootstrap_frames') = [1 10]; 
else 
    assert(false)
end