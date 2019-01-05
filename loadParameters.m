function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map;  

parameters('harris_r')              = 9;    % Harris Gaussian filter size (radius).  
parameters('harris_kappa')          = 0.08; % Harris keypoint extraction parameter. 
parameters('harris_r_sup')          = 8;    % radius of suppressing adjacent keypoints. 
parameters('harris_r_desc')         = 9;    % radius of patch descriptor. 
parameters('match_lambda')          = 5;    % matching threshold (multiplier of smallest SSD match). 

parameters('fund_num_iter')         = 200;  % max #iterations ransac for fundamental matrix estimation. 
parameters('fund_max_error')        = 1;    % max reprojection error for fundamental matrix estimation.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% KITTI  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ds == 0
parameters('bootstrap_frames')      = [1 3]; 
parameters('init_num_kps')          = 1000; % #keypoints for initialization. 
parameters('cont_num_kps')          = 200;  % #keypoints for candidate search in contop.
parameters('reinit_min_kps')        = 30;   % below #keypoints reinitialize.  

parameters('klt_max_bierror')       = inf;  % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 6;    % minimal # of RANSAC inliers to be valid pose estimate.                               
parameters('p3p_num_iter')          = 80;   % #iterations ransac for p3p.                                      
parameters('p3p_max_error')         = 5;    % max reprojection error to be p3p ransac inlier. 

parameters('search_min_dis')        = 10;   % minimal distance from new kp candidates to existing kps. 

parameters('counter_max')           = 10;   % maximal counter value to keep correspondence. 
parameters('counter_cand_max')      = 10;   % maximal counter value to keep candidate. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Malaga %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif ds == 1
parameters('bootstrap_frames')      = [1 3]; 
parameters('init_num_kps')          = 800;  % #keypoints for initialization. 
parameters('cont_num_kps')          = 200;  % #keypoints for candidate search in contop.
parameters('reinit_min_kps')        = 30;   % below #keypoints reinitialize.  

parameters('klt_max_bierror')       = inf;  % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 6;    % minimal # of RANSAC inliers to be valid pose estimate.                               
parameters('p3p_num_iter')          = 80;   % #iterations ransac for p3p.                                      
parameters('p3p_max_error')         = 5;    % max reprojection error to be p3p ransac inlier. 

parameters('search_min_dis')        = 10;   % minimal distance from new kp candidates to existing kps.

parameters('counter_max')           = 10;   % maximal counter value to keep correspondence. 
parameters('counter_cand_max')      = 10;   % maximal counter value to keep candidate. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Parking %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif ds == 2
parameters('bootstrap_frames')      = [10 13];
parameters('init_num_kps')          = 200;  % #keypoints for initialization. 
parameters('cont_num_kps')          = 200;  % #keypoints for candidate search in contop.
parameters('reinit_min_kps')        = 30;   % below #keypoints reinitialize.  
 
parameters('klt_max_bierror')       = inf;  % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 6;    % minimal # of RANSAC inliers to be valid pose estimate.                               
parameters('p3p_num_iter')          = 80;   % #iterations ransac for p3p.                                      
parameters('p3p_max_error')         = 5;    % max reprojection error to be p3p ransac inlier. 

parameters('search_min_dis')        = 10;   % minimal distance from new kp candidates to existing kps. 

parameters('counter_max')           = 10;   % maximal counter value to keep correspondence. 
parameters('counter_cand_max')      = 10;   % maximal counter value to keep candidate. 
else 
    assert(false)
end