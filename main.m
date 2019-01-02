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
    kitti_path = 'datasets/kitti/'; 
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 454;%0;   
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2))); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame
        img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end     
elseif ds == 1
    malaga_path = 'datasets/malaga/'; 
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2)); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame
        img = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end
elseif ds == 2
    parking_path = 'datasets/parking/'; 
    last_frame = 549;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    % Load bootstrap images. 
    bootstrap_frames = params('bootstrap_frames'); 
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    imgs_bootstrap = [img0; img1]; 
    % Load continous operation images. 
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), ...
                        last_frame-bootstrap_frames(2))); 
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame    
        img = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end
else
    assert(false);
end

%% Bootstrap Initialization.
disp("Starting bootstrapping initialization ..."); 

% Load parameter. 
patch_size  = params('patch_size'); 
kappa       = params('harris_kappa'); 
r_sup       = params('r_suppression'); 
num_kp_init = params('num_keypoints_init'); 
r_desc      = params('r_desciptor');
lambda      = params('match_lambda'); 

% Detect and extract features in img0.
scores_prev = harris(img0, patch_size, kappa);
kp_prev = selectKeypoints(scores_prev, num_kp_init, r_sup);
desc_prev = describeKeypoints(img0, kp_prev, r_desc);
% Detect and extract features in img1.
harris_scores = harris(img1, patch_size, kappa);
kp = selectKeypoints(harris_scores, num_kp_init, r_sup);
desc = describeKeypoints(img1, kp, r_desc);
% Match them!
matches = matchDescriptors(desc, desc_prev, lambda);
[~, query_indices, match_indices] = find(matches);
Pq = kp(:, query_indices);
Pdb = kp_prev(:, match_indices); 
fprintf('... number of matches: %d\n', size(Pq,2)); 

% Use the ground truth to set the scale of this bootstrap transformation.
t_norm_base = 0.1294;   % ToDo: Take this from ground truth.

% Estimate the F by ransac, extract the correct R,t & triangulate landmarks.
[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pdb,Pq, K, t_norm_base);  

%% Plotting - Debugging. 
% figure
% plotMatches(flipud(query_kps), flipud(database_kps), img0); 
% plotPointCloud(X); 

%% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; [0,0,0,1]];  
state.P = Pq;
state.Pdb = Pdb; 
state.X = X; 
state.Xin = X;  
trajectory = [state];  %#ok<NBRAK>

disp("Initial transformation: "); 
disp([R_CW t3_CW]); 
fprintf('Initial number of matches: %d\n', size(Pq,2));

%% Continuous operation.
figure; 
for i = 2:size(imgs_contop,3)
    fprintf('\nProcessing frame %d\n=====================\n', i);
    img_prev = imgs_contop(:,:,i-1); 
    img = imgs_contop(:,:,i); 
    
    % Get last state information. 
    state_prev = trajectory(end); 
    T_prev = state_prev.T; 
    P_prev = state_prev.P; 
    X_prev = state_prev.X; 

    % Track keypoints with MATLAB's KLT algorithm.
    disp("KL Tracking ...");   
    point_tracker = vision.PointTracker('MaxBidirectionalError',inf);
    initialize(point_tracker, fliplr(P_prev'), img_prev);
    [points,validity] = point_tracker(img);
    Pq = [];
    Pdb = [];
    X = [];
    for l=1:size(validity,1)
        if(validity(l)==1)
            Pq = [Pq,(fliplr(points(l,:)))']; %#ok<AGROW>
            Pdb = [Pdb,P_prev(:,l)]; %#ok<AGROW>
            X = [X,X_prev(:,l)]; %#ok<AGROW>
        end
    end
    fprintf('KLT number of tracked keypoints: %d\n', nnz(validity));
    
    % Either use P3P to track points or directly triangulate new landmarks.
    do_triangulate = false; 
    if nnz(validity) >= params('min_num_p3p')
        % Find new camera pose with MATLAB's P3P function.
        image_points = fliplr(Pq');
        world_points = X(1:3,:)';
        K_matlab = [K(1,1),0,0;0,K(2,2),0;K(1,3),K(2,3),1]; 
        camera_params = cameraParameters('IntrinsicMatrix', K_matlab);
        [world_orientation,world_location,status] = estimateWorldCameraPose(...
            image_points, world_points, camera_params,...
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
        if size(X,2)<params('min_num_landmarks')
            do_triangulate = true; 
            fprintf('Few landmarks, '); 
        elseif nnz(status)<params('min_num_inliers')
            do_triangulate = true; 
            fprintf('P3P bad status, '); 
        %elseif abs(mean(Pq(2,:))-320)>32 || abs(mean(Pq(1,:))-240)>24
        %    do_triangulate = true; 
        %    fprintf('Landmarks badly distributed, '); 
        end
    else
        do_triangulate = true; 
        fprintf('Not enough KLT tracked points, '); 
    end
    
    % Triangulate new landmarks if necessary. 
    if do_triangulate
        fprintf('triangulating new landmarks...\n')
        % Load parameter. 
        patch_size  = params('patch_size'); 
        kappa       = params('harris_kappa'); 
        r_sup       = params('r_suppression'); 
        num_kp_cont = params('num_keypoints_cont'); 
        r_desc      = params('r_desciptor');
        lambda      = params('match_lambda'); 
        % Find most distant database image with sufficient matches, i.e. 
        % start with large distant and decrease it until either the 
        % previous (shift = 1) image or a sufficient number of matches 
        % is obtained. 
        do_research = true; 
        shift  = min(params('triang_max_baseline'), size(trajectory,1)-1); 
        while do_research
            img_db  = imgs_contop(:,:,i-shift); 
            T_CW_db = trajectory(end-shift).T; 
            % Detect and extract new features in previous frames.
            scores_prev = harris(img_db, patch_size, kappa);
            kp_prev = selectKeypoints(scores_prev, num_kp_cont, r_sup); 
            desc_prev = describeKeypoints(img_db, kp_prev, r_desc);
            % Detect and extract features in the current frame.
            harris_scores = harris(img, patch_size, kappa);
            kp = selectKeypoints(harris_scores, num_kp_cont, r_sup);
            desc = describeKeypoints(img, kp, r_desc);      
            % Match them!
            matches = matchDescriptors(desc, desc_prev, lambda);
            [~, query_indices, match_indices] = find(matches);
            Pq_new = kp(:, query_indices);
            Pdb_new = kp_prev(:, match_indices);   
            fprintf('... number of matches: %d\n', size(Pq_new,2)); 
            % Check whether matches are sufficiently many. 
            if size(Pq_new,2) > params('min_num_landmarks')||shift == 1
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
        [~,~,X_new,~] = estimateTrafoFund(Pdb_new, Pq_new, K, norm(t3_CcCp));
        % OBSERVATION: We have a problem here that there is noise in Z and Y
        % direction. Without it, the difference was exactly 0.5, as GT
        % suggests, but with GT the distance was 1.36. This is okay because
        % we triangulated a completely new pointcloud. But this will
        % inevitably lead to drift and errors in the long term.
                
        % Transform old keypoints in current camera frame and append new 
        % and old landmarks. 
        X_old = T_CW_db*X;        
        X_cand = [X_old X_new];
        Pq_cand = [Pq Pq_new]; 
        Pdb_cand = [Pdb Pdb_new]; 
        
        % Discard landmarks that are very far away (as they most probably
        % are created by triangulation depth errors), behind the camera 
        % in an unreliable angle. 
        status_close   = X_cand(3,:)<params('landmarks_max_dis'); 
        status_infront = X_cand(3,:)>0; 
        status = status_close & status_infront; 
        fprintf('... number of distance landmarks: %d\n', nnz(~status));
        
        % Use the T_CW of the previous frame to bring back the 
        % pointcloud to the world frame. 
        X = inv(T_CW_db)*X_cand(:, status);  
        Pq = Pq_cand(:, status); 
        Pdb = Pdb_cand(:, status); 
        status = true(size(X,2),1); 
        fprintf('... number of new landmarks: %d\n', size(X,2));
    end
        
    % Renew state and add to trajectory.
    state = struct; 
    state.T = [R_CW t3_CW; [0,0,0,1]];  
    state.P = Pq; 
    state.Pdb = Pdb; 
    state.X = X; 
    state.Xin = X(:,status); 
    trajectory = [trajectory; state]; %#ok<AGROW>
    fprintf('Position: x=%f, y=%f, z=%f\n', ...
            state.T(1,4), state.T(2,4), state.T(3,4));
    % Plotting. 
    plotOverall(img, trajectory);
    %plotMatches(flipud(Pq), flipud(Pdb), img); 
    pause(0.01);
end
