function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map; 
parameters('patch_size') = 9;           % radius of patch to track. 
parameters('harris_kappa') = 0.08;
parameters('num_keypoints') = 200;
parameters('r_suppression') = 8;
parameters('r_desciptor') = 9;
parameters('match_lambda') = 5;
parameters('klt_num_iters') = 50;       % number of warping iterations.
parameters('klt_lambda') = 0.1;         % bidirectional error threshold.
parameters('contop_resize') = 0.25;     % contop image resize factor. 

if ds == 0
    parameters('bootstrap_frames') = [1 2];     
elseif ds == 1
    parameters('bootstrap_frames') = [1 2]; 
elseif ds == 2
    parameters('bootstrap_frames') = [1 10]; 
else 
    assert(false)
end