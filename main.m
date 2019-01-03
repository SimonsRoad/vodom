%##########################################################################
% vodom - Visual Odometry Pipeline
% Nikhilesh Alatur, Simon Schaefer
%##########################################################################
clear all; close all; clc;  %#ok<CLALL>

addpath(genpath('functions/'));

disp("################################")
disp("vodom - Visual Odometry Pipeline")
disp("Nikhilesh Alatur, Simon Schaefer")
disp("################################")
%% Choose and load dataset. 
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

% Parameters. 
params = loadParameters(ds); 

% Load dataset - Images and ground truth. 
ground_truth = NaN; 
imgs_bootstrap = []; 
imgs_contop = [];
disp("Loading dataset images ..."); 
if ds == 0
    ground_truth = load(['datasets/kitti/' '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 454;%0;   
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = imread(['datasets/kitti/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread(['datasets/kitti/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2))); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame
        img = imread(['datasets/kitti/00/image_0/' sprintf('%06d.png',i)]);
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end   
    % Adapt ground truth to initialization. 
        ground_truth = ground_truth(bootstrap_frames(2)+1:last_frame, :);
elseif ds == 1
    images = dir(['datasets/malaga/' ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = rgb2gray(imread(['datasets/malaga/' ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread(['datasets/malaga/' ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2)); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame
        img = rgb2gray(imread(['datasets/malaga/' ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end
elseif ds == 2
    last_frame = 549;
    K = load([parking_path '/K.txt']);
    ground_truth = load(['datasets/parking/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = rgb2gray(imread(['datasets/parking/' ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread(['datasets/parking/' ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2))); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame    
        img = im2uint8(rgb2gray(imread(['datasets/parking/' ...
            sprintf('/images/img_%05d.png',i)])));
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end
    % Adapt ground truth to initialization. 
    ground_truth = ground_truth(bootstrap_frames(2)+1:last_frame, :);
else
    assert(false);
end

K_matlab = [K(1,1),0,0;0,K(2,2),0;K(1,3),K(2,3),1]; 
K_matlab = cameraParameters('IntrinsicMatrix', K_matlab);

%% Bootstrap Initialization.
disp("Starting bootstrapping initialization ..."); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Bootstrap Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[P, Pdb] = harrisMatching(img1, img0, ...
                          params('r_desciptor'), params('match_ratio')); 
fprintf('... number of matches: %d\n', size(P,2)); 

% Use the ground truth to set the scale of this bootstrap transformation.
t_norm_base = 1.0;  

% Estimate the F by ransac, extract the correct R,t & triangulate landmarks.
[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pdb, P, ...
                           K, t_norm_base, params('num_iter_fund'));  

% stereo_params = stereoParameters(K_matlab,K_matlab,R_CW,t3_CW); 
% X = triangulate(Pdb', P', stereo_params); 
% X = X'; 

% Plotting. 
figure
%plotMatches(P, Pdb, img1);
plotPointCloud(X(:,25), P(:,25), img1); 

% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; [0,0,0,1]]; 
state.P = P;
state.X = X; 
state.Xin = X;  
trajectory = [state];  %#ok<NBRAK>

disp("Initial transformation: "); 
disp([R_CW t3_CW]); 
fprintf('Initial number of matches: %d\n', size(P,2));

%% Continuous operation.
figure(2); 
for i = 2:size(imgs_contop,3)
    fprintf('\nProcessing frame %d\n=====================\n', i);
    img_prev = imgs_contop(:,:,i-1); 
    img = imgs_contop(:,:,i); 
    
    % Get last state information. 
    state_prev = trajectory(end); 
    T_prev   = state_prev.T; 
    P_prev   = state_prev.P; 
    X_prev   = state_prev.X; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% KLT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Track keypoints with MATLAB's KLT algorithm.
    disp("KL Tracking ...");   
    point_tracker = vision.PointTracker(...
        'MaxBidirectionalError', params('klt_max_bierror'));
    initialize(point_tracker, fliplr(P_prev'), img_prev);
    [points,validity] = point_tracker(img);
    P = [];
    Pdb = [];
    X = [];
    for l=1:size(validity,1)
        if(validity(l)==1)
            P = [P,(fliplr(points(l,:)))']; %#ok<AGROW>
            Pdb = [Pdb,P_prev(:,l)]; %#ok<AGROW>
            X = [X,X_prev(:,l)]; %#ok<AGROW>
        end
    end
    fprintf('KLT number of tracked keypoints: %d\n', nnz(validity));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% P3P %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Either use P3P to track points or directly triangulate new landmarks.
    do_triangulate = false; 
    if nnz(validity) >= params('min_num_p3p')
        % Find new camera pose with MATLAB's P3P function.
        image_points = fliplr(P');
        world_points = X(1:3,:)';
        [world_orientation,world_location,status] = estimateWorldCameraPose(...
            image_points, world_points, K_matlab,...
            'MaxReprojectionError', params('max_reproj_error_p3p'), ...
            'MaxNumTrials', params('num_iter_p3p'));
        % Convert T_WC->T_CW.
        R_WC = world_orientation;
        t_WC = world_location';
        T_WC = [R_WC,t_WC;[0,0,0,1]];
        T_CW = inv(T_WC);
        R_CW = T_CW(1:3,1:3);
        t3_CW = T_CW(1:3,4);
        assert(isequal(size([R_CW t3_CW]), [3,4])); 
        % Check whether it is necessary to triangulate new landmarks, 
        % after motion has been estimated.
        if size(X,2)<params('reinit_min_num_landmk')
            do_triangulate = true; 
            fprintf('Few landmarks, '); 
        elseif nnz(status)<params('reinit_min_num_inlier')
            do_triangulate = true; 
            fprintf('P3P bad status, '); 
        %elseif abs(mean(P(2,:))-320)>32 || abs(mean(P(1,:))-240)>24
        %    do_triangulate = true; 
        %    fprintf('Landmarks badly distributed, '); 
        %elseif abs(var(P(2,:)))<params('reinit_min_landmk_var') || ...
        %       abs(var(P(1,:)))<params('reinit_min_landmk_var')
        %   do_triangulate = true; 
        %   fprintf('Landmarks badly distributed, '); 
        end
    else
        break
        do_triangulate = true; 
        fprintf('Not enough KLT tracked points, '); 
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Re-Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Triangulate new landmarks if necessary. 
    do_triangulate = false;  %HACK
    if do_triangulate
        fprintf('triangulating new landmarks...\n')
        % Find most distant database image with sufficient matches, i.e. 
        % start with large distant and decrease it until either the 
        % previous (shift = 1) image or a sufficient number of matches 
        % is obtained. 
        do_research = true; 
        shift = min(params('reinit_max_base'),max(size(trajectory,1)-1));
        shift = max(shift, 1); 
        while do_research
            img_db  = imgs_contop(:,:,i-shift); 
            T_CW_db = trajectory(end-shift+1).T; 
            [P_new, Pdb_new] = harrisMatching(img1, img0, ...
                               params('r_desciptor'), params('match_ratio')); 
            fprintf('... number of matches: %d\n', size(P_new,2)); 
            % Check whether matches are sufficiently many. 
            if size(P_new,2) > params('reinit_min_num_landmk')||shift == 1
                do_research = false; 
            else
                shift = shift - 1; 
            end
        end
        % Find the norm of the transform from the last to the current
        % frame. Required for scaling the translation vector 
        % from estimateTrafoFund.
        T_CcCp = [R_CW t3_CW; [0,0,0,1]]*inv(T_CW_db);
        t3_CcCp = T_CcCp(1:3,4);
     
        % Triangulate new landmarks.
        % OBSERVATION: We have a problem here that there is noise in Z and Y
        % direction. Without it, the difference was exactly 0.5, as GT
        % suggests, but with GT the distance was 1.36. This is okay because
        % we triangulated a completely new pointcloud. But this will
        % inevitably lead to drift and errors in the long term.
        [~,~,X_new,~] = estimateTrafoFund(Pdb_new, P_new, ...
                        K, norm(t3_CcCp), params('num_iter_fund'));
   
        % Transform old keypoints in current camera frame and append new 
        % and old landmarks. 
        X_old = T_CW_db*X;        
        X_cand = [X_old X_new];
        P_cand = [P P_new]; 
        
        % Discard landmarks that are very far away (as they most probably
        % are created by triangulation depth errors), behind the camera 
        % in an unreliable angle. 
        status_close   = X_cand(3,:)<params('discard_lm_max_dis'); 
        status_infront = X_cand(3,:)>0; 
        status = status_close & status_infront; 
        fprintf('... number of distance landmarks: %d\n', nnz(~status));        
        % Use the T_CW of the previous frame to bring back the 
        % pointcloud to the world frame. 
        X = inv(T_CW_db)*X_cand(:, status);  
        P = P_cand(:, status); 
        status = true(size(X,2),1); 
        fprintf('... number of new landmarks: %d\n', size(X,2));
    end
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Post-Processing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check whether any landmarks are still available, otherwise take 
    % previous ones. 
    if size(X, 2) < params('post_min_num_landmk')
       X = X_prev; 
       disp('Taking previous set of landmarks');
    end 
        
    % Renew state and add to trajectory.
    state = struct; 
    state.T = [R_CW t3_CW; [0,0,0,1]];  
    state.P = P; 
    state.X = X; 
    state.Xin = X(:,status); 
    trajectory = [trajectory; state]; %#ok<AGROW>
    fprintf('Position: x=%f, y=%f, z=%f\n', ...
            state.T(1,4), state.T(2,4), state.T(3,4));
    % Plotting. 
    %plotOverall(img, trajectory);
    plotKPs(P, img); 
    pause(0.01);
end

%% Post Bundle-Adjustment.
if isnan(ground_truth)
    disp("No groud_truth available !");
else
    disp("Compare trajectory to ground_truth ..."); 
    num_points = size(trajectory,1); 
    % Determine groundtruth and trajectory positions. 
    p_W_GT = [ground_truth(1:num_points, :)'; zeros(1,num_points)]; 
    p_W_E  = zeros(3,num_points); 
    for i = 1:num_points
        p_W_E(:,i) = [trajectory(i).T(1,4); trajectory(i).T(3,4); 0.0];
    end
    % Align groundtruth and estimate to resolve scale and rotational errors. 
    p_W_E_aligned = alignEstimateToGroundTruth(p_W_GT, p_W_E); 
    % Plot groundtruth vs aligned trajectory. 
    figure(3)
    plotTrajectoryGT(p_W_E, p_W_E_aligned, p_W_GT); 
end