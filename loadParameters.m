function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map;  
 
parameters('r_desciptor')        = 9;    % pixel radius of patch descriptor. 
parameters('match_ratio')        = 0.8;  % rejecting ambiguous matches. 
                                         % Increase to return more matches.
parameters('num_iter_fund')      = 1000; % max #iterations ransac for 
                                         % fundamental matrix estimation. 

parameters('klt_max_bierror')    = inf;  % max. bidirectional error for KLT.                        
                                         
parameters('min_num_p3p')         = 50;  % min #point matches for p3p.                                          
parameters('num_iter_p3p')        = 1000;% max #iterations ransac for p3p.                                      
parameters('max_reproj_error_p3p') = 1;  % max reprojection error for p3p 
                                         % ransac trafo estimate. 

parameters('reinit_min_num_landmk') = 40;% min #landmarks in contop.
parameters('reinit_min_num_inlier') = 25;% min #inliers of P3P in contop.
parameters('reinit_min_landmk_var') = 10;% min variance of landmark distrib.
parameters('reinit_max_base')       = 4; % max distance between db & q img
                                         % to triangulate new landmarks. 
parameters('discard_lm_max_dis')    = 20;% max distance cam-landmark. 

parameters('post_min_num_landmk')   = 5; % min #landmarks to append to traj.

if ds == 0
    parameters('bootstrap_frames')   = [1 4];    
    parameters('num_keypoints_init') = 1;  % #keypoints for initialization.
    parameters('num_keypoints_cont') = 200;  % #keypoints for contop.
elseif ds == 1
    parameters('bootstrap_frames')   = [1 2]; 
elseif ds == 2
    parameters('bootstrap_frames')   = [40 50]; % We need to start here, 
        % as we get >40 features after frame 40, but only 20 in the 
        % beginning, furthermore a large step ensures small depth error.
    parameters('num_keypoints_init') = 800;  % #keypoints for initialization.
    parameters('num_keypoints_cont') = 500;  % #keypoints for contop.
else 
    assert(false)
end