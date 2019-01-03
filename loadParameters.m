function parameters = loadParameters(ds)
% Load parameters for each dataset (0: KITTI, 1: Malaga, 2: parking).
parameters = containers.Map; 
parameters('num_keypoints_init') = 800;  % #keypoints for initialization. 
parameters('num_keypoints_cont') = 500;  % #keypoints for contop.

parameters('patch_size') = 9;            % radius of patch to track. 
parameters('harris_kappa') = 0.08;       % Harris kp extraction parameter. 
parameters('r_suppression') = 8;         % radius of suppressing adj. kps. 
parameters('r_desciptor') = 9;           % pixel radius of patch descriptor. 
parameters('match_lambda') = 5;          % matching threshold. 

parameters('min_num_landmarks') = 40;    % min #landmarks in contop.
parameters('min_num_inliers') = 25;      % min #inliers of P3P in contop.
parameters('landmarks_max_dis') = 20;    % max distance cam-landmark. 
parameters('triang_max_baseline') = 4;   % max distance between db & q img
                                         % to triangulate new landmarks. 

parameters('klt_max_bierror') = inf;     % max. bidirectional error for KLT.                        
                                         
parameters('min_num_p3p') = 10;          % min #point matches for p3p.                                          
parameters('num_iter_p3p') = 500;        % max #iterations ransac for p3p.                                      
parameters('max_reproj_error_p3p') = 8;  % max reprojection error for p3p 
                                         % ransac trafo estimate. 

if ds == 0
    parameters('bootstrap_frames') = [1 2];    
elseif ds == 1
    parameters('bootstrap_frames') = [1 2]; 
elseif ds == 2
     % We need to start here, as we get >40 features after frame 40, 
     % but only 20 in the beginning, furthermore a large step 
     % ensures small depth error.
    parameters('bootstrap_frames') = [40 50];
else 
    assert(false)
end

% Old parameters. 
% parameters('klt_use_matlab') = true;    % use matlab KLT implementation.
% parameters('klt_num_iters') = 50;       % number of warping iterations.
% parameters('klt_lambda') = 0.1;         % bidirectional error threshold.
% parameters('klt_min_num_kps') = 50;     % min. #kps for KLT tracking. 
% parameters('ransac_num_iter') = 1000;   % #ransac optimization iterations.
% parameters('ransac_pixel_tolerance') = 50; % acceptable pixel reproj. error.  
% parameters('ransac_min_inlier') = 30;   % min. #acceptable ransac inlier.
% parameters('ransac_num_samples') = 3;   % sample size for ransac.
% parameters('min_angle_landmarks') = 1.0; % minimal angle between detection  
%                                          % and current frame wrt landmark.
% parameters('max_angle_landmarks') = 6.8; % maximal angle between detection  
%                                          % and current frame wrt landmark.