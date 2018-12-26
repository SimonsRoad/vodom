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

%% Bootstrap
disp("Starting bootstrapping initialization ..."); 
[kp1, dc1] = features(imgs_bootstrap(1:480,:), params);
[kp2, dc2] = features(imgs_bootstrap(480:end,:), params);
matches = matching(dc1, dc2, params);
[qm_kps, dbm_kps] = matches2kps(matches, kp2, kp1);
[R_CW, t3_CW, cloud] = estimateTrafo(qm_kps, dbm_kps, K); 

% Plotting - Debugging. 
plotMatches(qm_kps, dbm_kps, imgs_bootstrap(1:480,:)); 
%plotPointCloud(cloud); 

% Assign initial state. 
state = struct; 
state.pose = [R_CW t3_CW; ones(1,4)]; 
trajectory = [state];  %#ok<NBRAK>

disp("Initial transformation: "); 
disp([R_CW t3_CW]); 

%% Continuous operation
figure; 
for i = 2:size(imgs_contop,3)
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    img_prev = imgs_contop(:,:,i-1); 
    img = imgs_contop(:,:,i); 
    % Resizing images (and keypoints !) to improve performance. 
    img_prev = imresize(img_prev, params('contop_resize')); 
    img = imresize(img, params('contop_resize')); 
    kps = dbm_kps .* params('contop_resize'); 
    % Apply KLT for every keypoint. 
    dkp = zeros(size(kps));
    keep = true(1, size(kps, 2));
    for j = 1:size(kps,2)
        [delta, keep] = trackKLT(img_prev, img, kps(:,i)', params);
    end
    kpold = kps(:, keep);
    kps = kps + dkp;
    kps = kps(:, keep);
    % Plotting. 
    %plotMovingKPs(kpold, kps, img); 
    pause(0.01);
    break; 
end
