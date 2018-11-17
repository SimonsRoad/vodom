%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Visual odometry pipeline implementation for several datasets (KITTI, 
% Malaga, parking), based on a relative transformation between subsequent
% images calculated on the basis of Harris feature, patch descriptors
% and the fundamental matrix (including RANSAC outlier suppression). 
% T - camera world transformation.  
% S - set of pixel coordinates. 
% C - set of correspondences. 
% D - set of keypoint descriptors. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;  %#ok<CLALL>

% Choose dataset (0: KITTI, 1: Malaga, 2: parking). 
global dataset; 
dataset = 1; 

% Set the two bootstrap frames. 
bootstrap_frames = [1 3]; 

% Declare dataset paths. 
kitti_path = 'datasets/kitti/'; 
malaga_path = 'datasets/malaga-urban-dataset-extract-07/'; 
parking_path = 'datasets/parking/';

% Dataset runtime parameters. 
global params; 
params = containers.Map; 
params('feature_quality') = [0.001 0.001 0.001];
params('feature_max_num') = [100, 100, 100]; 
params('descriptor_size') = [7 7 7]; 

% Add code subdirectories. 
addpath('features'); 
addpath('miscellaneous'); 
addpath('multiview'); 
addpath('plotting'); 

%% Load choosen dataset and camera parameter. 
if dataset == 0
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif dataset == 1
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif dataset == 2
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

range = (bootstrap_frames(2)+1):last_frame;
num_frames = size(range,2) + 1; 
% Initialise storage matrices for path and descriptors.
cam_positions = zeros(4,num_frames);    % camera positions (homo.coords).
cam_positions(4,:) = 1; 
Ts = zeros(3,4,num_frames);             % camera transformations [R|T].
Ds = cell(num_frames);                  % descriptors cell array.  
Ss = cell(num_frames);                  % keypoint pixel coordinates. 

num_tracked_keypoints = zeros(1,num_frames); % number of tracked keypoints
                                             % for each frame. 

%% Bootstrap initialization. 
if dataset == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif dataset == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif dataset == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

% Detect features in current image. 
[S1,D1] = keypoints(img0, param('feature_max_num'), ...
        param('feature_quality'), param('descriptor_size')); 
[S2,D2] = keypoints(img0, param('feature_max_num'), ...
        param('feature_quality'), param('descriptor_size')); 
% Find matched features. 
Cindex = matchDescriptors(D1,D2); 
C1 = S1(:,Cindex(:,1)); 
C2 = S2(:,Cindex(:,2)); 
% Estimate relative transformation by determining fundamental matrix. 
[Rwc,twc] = estimateTransformation(C1,C2,K);
Twc = [Rwc,twc]; 
% Previous = current association.  
Ds{1} = D2;
Ss{1} = S2; 
Ts(:,:,1) = Twc; 
num_tracked_keypoints(1) = size(Cindex,1); 

%% Continuous operation
for k = 2:num_frames
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    % Select next image from dataset. 
    i = range(k-1); 
    if dataset == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif dataset == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif dataset == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Detect features in current image. 
    [S,D] = keypoints(image, param('feature_max_num'), ...
        param('feature_quality'), param('descriptor_size')); 
    % Find matched features. 
    Cindex = matchDescriptors(D,Ds{k-1}); 
    C1 = S1(:,Cindex(:,1)); 
    C2 = S2(:,Cindex(:,2));  
    % Estimate relative transformation by determining fundamental matrix. 
    [R,t] = estimateTransformation(C1,C2,K);
    Twc = [R,t]; 
    cam_positions(:,k) = [Twc;zeros(1,3),1]*cam_positions(:,k-1); 
    % Previous = current association.  
    prev_img = image;
    Ds{k} = D;
    Ss{k} = S; 
    Ts(:,:,k) = Twc; 
    num_tracked_keypoints(k) = size(Cindex,1); 
    % Refresh continous plot. Pause to make sure that plots refresh.   
    plotContinousStatus(image,C1,C2,cam_positions(1:3,1:k), ...
                        num_tracked_keypoints(1:k)); 
    pause(0.01);
end
