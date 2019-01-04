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
parameters('cont_num_kps')          = 400;  % #keypoints for contop.

parameters('klt_max_bierror')       =inf;   % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 10;   % min #point matches for p3p.                                          
parameters('p3p_num_iter')          = 2000; % max #iterations ransac for p3p.                                      
parameters('p3p_max_reproj_error')  = 8;    % max reprojection error for p3p ransac trafo estimate. 

parameters('reinit_min_num_landmk') = 0;    % min #landmarks in contop.
parameters('reinit_min_num_inlier') = 0;	% min #inliers of P3P in contop.
parameters('reinit_min_kp_var')     = 4 ;   % min variance of landmark distrib.
parameters('reinit_max_base')       = 4;    % max distance between db & q img to triangulate new landmarks. 
parameters('discard_lm_max_dis')    = 200;  % max distance cam-landmark. 

parameters('post_min_num_landmk')   = 5;    % min #landmarks to append to traj.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Malaga %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif ds == 1
parameters('bootstrap_frames')      = [1 3]; 
parameters('init_num_kps')          = 800;  % #keypoints for initialization. 
parameters('cont_num_kps')          = 500;  % #keypoints for contop.

parameters('klt_max_bierror')       =inf;   % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 10;   % min #point matches for p3p.                                          
parameters('p3p_num_iter')          = 2000; % max #iterations ransac for p3p.                                      
parameters('p3p_max_reproj_error')  = 8;    % max reprojection error for p3p ransac trafo estimate. 

parameters('reinit_min_num_landmk') = 40;   % min #landmarks in contop.
parameters('reinit_min_num_inlier') = 10;   % min #inliers of P3P in contop.
parameters('reinit_min_kp_var')     = 4 ;   % min variance of landmark distrib.
parameters('reinit_max_base')       = 4;    % max distance between db & q img to triangulate new landmarks. 
parameters('discard_lm_max_dis')    = 200;  % max distance cam-landmark. 

parameters('post_min_num_landmk')   = 5;    % min #landmarks to append to traj.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Parking %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif ds == 2
parameters('bootstrap_frames')      = [1 3];
parameters('init_num_kps')          = 200;  % #keypoints for initialization. 
parameters('cont_num_kps')          = 200;  % #keypoints for contop.
 
parameters('klt_max_bierror')       =inf;   % max. bidirectional error for KLT.                        
                                         
parameters('p3p_min_num')           = 10;   % min #point matches for p3p.                                          
parameters('p3p_num_iter')          = 2000; % max #iterations ransac for p3p.                                      
parameters('p3p_max_reproj_error')  = 8;    % max reprojection error for p3p ransac trafo estimate. 

parameters('reinit_min_num_landmk') = 40;   % min #landmarks in contop.
parameters('reinit_min_num_inlier') = 20;   % min #inliers of P3P in contop.
parameters('reinit_min_kp_var')     = 4 ;   % min variance of landmark distrib.
parameters('reinit_max_base')       = 4;    % max distance between db & q img to triangulate new landmarks. 
parameters('discard_lm_max_dis')    = 200;  % max distance cam-landmark. 

parameters('post_min_num_landmk')   = 5;    % min #landmarks to append to traj.
else 
    assert(false)
end