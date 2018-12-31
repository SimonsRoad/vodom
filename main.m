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
    last_frame = 454;%0;    % DON'T KILL MY DUAL CORE PLS;)
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
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), last_frame));
    k = 1; 
    for i = (bootstrap_frames(2)+1):last_frame
        img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        imgs_contop(:,:,k) = img; 
        k = k + 1; 
    end     %BUG: WHITE IMAGES!
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
    for i = (bootstrap_frames(2)+1):last_frame    % HACK! More distance between frames

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
t_norm_base = 0.1294;%ground_truth(bootstrap_frames(2)) - ground_truth(bootstrap_frames(1));
[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pdb,Pq, K, t_norm_base);  % BUGFIX: Pdb is P1 and Pq is P2 in this function!

% figure; ax = axes;
% showMatchedFeatures(img0,img1,fliplr(Pdb'),fliplr(Pq'),'montage','Parent',ax);
% title(ax, 'Candidate point matches');
% legend(ax, 'Matched points 1','Matched points 2');

%% Plotting - Debugging. 
% plotMatches(flipud(query_kps), flipud(database_kps), img0); 
% plotPointCloud(X); 

%% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; [0,0,0,1]];  %BUGFIX, homogenous coordinate transform 
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
    % Reinitialize feature points if below threshold
    % !!!NOTE: DON'T DO IT HERE! FIRST WE NEED A MOTION ESTIMATE BEFORE WE CAN
    % EVEN TRIANGULATE NEW LANDMARKS!!!
    disp("KL Tracking ..."); 
        
    pointTracker = vision.PointTracker('MaxBidirectionalError',inf);
    initialize(pointTracker, fliplr(P_prev'), img_prev);
    [points,validity] = pointTracker(img);
    Pq = [];
    Pdb = [];
    X = [];
    for l=1:size(validity,1)
        if(validity(l)==1)
            Pq = [Pq,(fliplr(points(l,:)))'];
            Pdb = [Pdb,P_prev(:,l)];
            X = [X,X_prev(:,l)];
        end
    end
%     figure; ax = axes;
% showMatchedFeatures(img_prev,img,fliplr(Pdb'),fliplr(Pq'),'montage','Parent',ax);
% title(ax, 'Candidate point matches');
% legend(ax, 'Matched points 1','Matched points 2');

    % !!!NOTE: KLT WORKS!      
    %4.a) Try MATLAB's P3P
    imagePoints = fliplr(Pq');
    worldPoints = X(1:3,:);
    worldPoints = worldPoints';
    intrinsics = cameraIntrinsics([K(1,1) K(2,2)],[K(1,3) K(2,3)],size(img0));
    K_intrinsics = [K(1,1),0,0;...
        0,K(2,2),0;...
        K(1,3),K(2,3),1];
    cameraParams = cameraParameters('IntrinsicMatrix', K_intrinsics);
        
    [worldOrientation,worldLocation,status] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams,...
        'MaxReprojectionError',10);
    % Convert.
    R_WC = worldOrientation;
    t_WC = worldLocation';
    T_WC = [R_WC,t_WC];
    T_WC = [T_WC;[0,0,0,1]];
    T_CW = inv(T_WC);
    R_CW = T_CW(1:3,1:3);
    t3_CW = T_CW(1:3,4);
        
    assert(isequal(size([R_CW t3_CW]), [3,4])); 
    
    % Triangulate new landmarks, AFTER motion has been estimated.
    centroid_x = mean(Pq(2,:));
    centroid_y = mean(Pq(1,:));
    if size(X,2)<45%or(abs(centroid_x-320)>32,abs(centroid_y-240)>24)%
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

        T_CcCp = [R_CW t3_CW; [0,0,0,1]]*inv(T_prev);
        t3_CcCp = T_CcCp(1:3,4);
        [~, ~, X_new, ~] = estimateTrafoFund(Pdb, Pq, K, norm(t3_CcCp));  % BUGFIX: Use the norm of the relative translation, not w.r.t world frame!
        % NOTE: The problem we have here, is the noise in Z and Y
        % direction. Without it, the difference was exactly 0.5, as GT
        % suggests, but with GT the distance was 1.36. This is okay because
        % we triangulated a completely new pointcloud. But this will
        % inevitably lead to drift and errors in the long term.
        
        X = inv(T_prev)*X_new;  % Use the T_CW of the previous frame to bring back everything to world frame.
    end
    
    % Renew state and add to trajectory.
    state = struct; 
    state.T = [R_CW t3_CW; [0,0,0,1]];  %BUGFIX 
    state.P = Pq; 
    state.Pdb = Pdb; 
    state.X = X; 
    trajectory = [trajectory; state];
    
    % Plotting. 
    plotOverall(img,img_prev, trajectory); 
    pause(0.01);
end
