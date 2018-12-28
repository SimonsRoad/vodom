%##########################################################################
% vodom - Visual Odometry Pipeline
% Nikhilesh Alatur, Simon Schaefer
%##########################################################################
clear all; clc;  %#ok<CLALL>

addpath('features/'); 
addpath('klt/'); 
addpath('plotting/'); 
addpath('ransac/'); 

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
[P1, dc1] = features(imgs_bootstrap(1:480,:), params, false);
[P2, dc2] = features(imgs_bootstrap(480:end,:), params, false);
matches = matching(dc1, dc2, params);
[Pq, Pdb] = matches2kps(matches, P2, P1);
[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan); 

% Plotting - Debugging. 
%plotMatches(qm_kps, dbm_kps, imgs_bootstrap(1:480,:)); 
%plotPointCloud(cloud); 

% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; ones(1,4)]; 
state.P = Pq;
state.Pdb = Pdb; 
state.X = X; 
trajectory = [state];  %#ok<NBRAK>

disp("Initial transformation: "); 
disp([R_CW t3_CW]); 
fprintf('Initial number of matches: %d\n', size(Pdb,2));

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
    if size(P_prev, 2) < params('klt_min_num_kps')
        disp("Reinitializing ... "); 
        [P1, dc1] = features(img_prev, params, true);
        [P2, dc2] = features(img, params, true);
        matches = matching(dc1, dc2, params);
        [Pq, Pdb] = matches2kps(matches, P2, P1);
        [R_CW, t3_CW, X, ~] = estimateTrafoFund(Pq, Pdb, K, t3norm); 
        fprintf('Reinitialization number of matches: %d\n', size(matches,2));
    % Otherwise apply KLT for every keypoint. 
    else
        disp("KL Tracking ..."); 
        [P, keep] = trackKLT(img_prev, img, P_prev, params); 
        % Estimate transformation from previous to current frame. 
        Pq = P(:,keep); 
        Pdb = P_prev(:,keep); 
        Xdb = X_prev(:,keep); 
        [R_CW, t3_CW, ~] = estimateTrafoP3P(Pdb, Xdb, K, t3norm, params); 
        X = X_prev;  
        fprintf('KLT number of tracked keypoints: %d\n', size(keep,2));
    end
    assert(isequal(size([R_CW t3_CW]), [3,4])); 
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
