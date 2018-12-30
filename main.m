%##########################################################################
% vodom - Visual Odometry Pipeline
% Nikhilesh Alatur, Simon Schaefer
%##########################################################################
clear all; close all; clc;  %#ok<CLALL>

addpath('features/'); 
addpath('klt/'); 
addpath('plotting/'); 
addpath('ransac/'); 

addpath('debug_new/'); %Debug: Functions 1:1 from exercise solution.

disp("################################")
disp("vodom - Visual Odometry Pipeline")
disp("Nikhilesh Alatur, Simon Schaefer")
disp("################################")
%% Choose and load dataset. 
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

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
    last_frame = 4540;
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
    imgs_contop = zeros(size(img0,1), size(img0,2), last_frame);
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
    imgs_contop = zeros(size(img0,1), size(img0,2), last_frame); 
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
    last_frame = 598;
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
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), last_frame)); 
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
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

harris_scores = harris(img0, harris_patch_size, harris_kappa);
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);
descriptors = describeKeypoints(img0, keypoints, descriptor_radius);
harris_scores_2 = harris(img1, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img1, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

[~, query_indices, match_indices] = find(matches);
Pq = keypoints_2(:, query_indices);
Pdb = keypoints(:, match_indices);     

[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan);

%% Plotting - Debugging. 
% plotMatches(flipud(query_kps), flipud(database_kps), img0); 
% plotPointCloud(X); 

%% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; ones(1,4)]; 
state.P = Pq;
state.Pdb = Pdb; 
state.X = X; 
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
    % Reinitialize feature points if below threshold.
    if false %size(P_prev, 2) < params('klt_min_num_kps')   % DEBUG: Don't use this for now.
        disp("Reinitializing ... ");        
        fprintf('Reinitialization number of matches: %d\n', size(matches,2));
    % Otherwise apply KLT for every keypoint. 
    else
        disp("KL Tracking ..."); 
        %[P, keep] = trackKLT(img_prev, img, (flipud(P_prev))', params); % BUG: Feeding [y x] instead of [x y]
%         [W, p_hist] = trackKLT(I_R, I, x_T, r_T, num_iters)
%         % Estimate transformation from previous to current frame. 
%         Pq = P(:,keep); 
%         Pdb = P_prev(:,keep); 
%         Xdb = X_prev(:,keep); 
%         %[R_CW, t3_CW, ~] = estimateTrafoP3P(Pq, Xdb, K, t3norm, params); % BUG: Something's wrong! Output from p3p is nowhere close to a rotation matrix!
% %         M = dlt(fliplr(Pq'), (Xdb(1:3,:))', K); % Because P3P is failing, just do DLT with the tracked keypoints and the landmark pointcloud.
% %         R_CW = M(:,1:3);
% %         t3_CW = M(:,4);
%         %[R_CW, t3_CW, ~, ~] = estimateTrafoFund(Pq, Pdb, K, nan);
        %X = Xdb; %BUGFIX: Xdb, instead of X_prev  
        %fprintf('KLT number of tracked keypoints: %d\n', nnz(keep));
        
        %#######################DON'T USE KLT##############################
        % 1.) Extract descriptors from database.
        descriptors = describeKeypoints(img_prev, P_prev, descriptor_radius);

        % 2.) Detect and extract features from current view (-> Query).
        harris_scores_2 = harris(img, harris_patch_size, harris_kappa);
        keypoints_2 = selectKeypoints(...
            harris_scores_2, num_keypoints, nonmaximum_supression_radius);
        descriptors_2 = describeKeypoints(img, keypoints_2, descriptor_radius);
        
        % 3.)Directly match the db descriptors to query descriptors.
        matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

        [~, query_indices, match_indices] = find(matches);
        Pq = keypoints_2(:, query_indices);
        Pdb = P_prev(:, match_indices);     
        X = X_prev(:,match_indices);
        
        % 4.)estimateTrafoFund without caring about pointcloud.
        [R_CW, t3_CW, ~, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan);
        
        % 4.) Concatenate.
        % !!!PROBLEM:Although this works reliably, because we estimated the
        % fundamental matrix independently from the point cloud, we don't
        % know the relation to the scale. If we had done P3P or DLT, we had
        % used directly the 3D(database)<->2D(query) relation and we would
        % have gotten the scale automatically. But now, we don't have the
        % scale!
        
        %###########################################################
    end
    assert(isequal(size([R_CW t3_CW]), [3,4])); 
    %#########################Triangulate new landmarks############
    % If too little features kept: Triangulate new landmarks, by using the
    % just computed R_CW, t_CW.
    if size(X,2)<30
        disp("Triangulating new landmarks...")
        harris_scores = harris(img_prev, harris_patch_size, harris_kappa);
        keypoints = selectKeypoints(...
        harris_scores, num_keypoints, nonmaximum_supression_radius);
        descriptors = describeKeypoints(img_prev, keypoints, descriptor_radius);
        
        harris_scores_2 = harris(img, harris_patch_size, harris_kappa);
        keypoints_2 = selectKeypoints(...
            harris_scores_2, num_keypoints, nonmaximum_supression_radius);
        descriptors_2 = describeKeypoints(img, keypoints_2, descriptor_radius);

        matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

        [~, query_indices, match_indices] = find(matches);
        Pq = keypoints_2(:, query_indices);
        Pdb = keypoints(:, match_indices);     

        [~, ~, X_new, ~] = estimateTrafoFund(Pq, Pdb, K, norm(t3_CW));
        
        X = inv(T_prev)*X_new;  % Use the T_CW of the previous frame to bring back everything to world frame.
    end    
    %######################################################################
    % Renew state and add to trajectory.
    state = struct; 
    state.T = [R_CW t3_CW; ones(1,4)]; 
    state.P = Pq; 
    state.Pdb = Pdb; 
    state.X = X; 
    trajectory = [trajectory; state]; 
    % Plotting. 
    plotOverall(img, trajectory); 
    pause(0.01);
end
