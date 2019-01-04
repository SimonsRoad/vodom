function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map;  
parameters('init_num_kps')         = 800;% #keypoints for initialization. 
parameters('cont_num_kps')         = 500;% #keypoints for contop.
 
parameters('harris_r')              = 9; % Harris Gaussian filter size. 
parameters('harris_kappa')        = 0.08;% Harris kp extraction parameter. 
parameters('harris_r_sup')          = 8; % radius of suppressing adj. kps. 
parameters('harris_r_desc')         = 9; % pixel radius of patch descriptor. 
parameters('match_lambda')          = 5; % matching threshold. 

parameters('fund_num_iter')        = 500;% max #iterations ransac for 
                                         % fundamental matrix estimation. 

parameters('klt_max_bierror')       =inf;% max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 10;% min #point matches for p3p.                                          
parameters('p3p_num_iter')        = 2000;% max #iterations ransac for p3p.                                      
parameters('p3p_max_reproj_error')  = 8;% max reprojection error for p3p 
                                         % ransac trafo estimate. 

parameters('reinit_min_num_landmk') = 40;% min #landmarks in contop.
parameters('reinit_min_num_inlier') = 25;% min #inliers of P3P in contop.
parameters('reinit_min_landmk_var') = 10;% min variance of landmark distrib.
parameters('reinit_max_base')       = 4; % max distance between db & q img
                                         % to triangulate new landmarks. 
parameters('discard_lm_max_dis')    = 20;% max distance cam-landmark. 

parameters('post_min_num_landmk')   = 5; % min #landmarks to append to traj.

if ds == 0
    parameters('bootstrap_frames')   = [1 10];    
elseif ds == 1
    parameters('bootstrap_frames')   = [1 2]; 
elseif ds == 2
    parameters('bootstrap_frames')   = [40 50]; % We need to start here, 
        % as we get >40 features after frame 40, but only 20 in the 
        % beginning, furthermore a large step ensures small depth error.
else 
    assert(false)
end