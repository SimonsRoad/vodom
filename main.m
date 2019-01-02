%##########################################################################
% vodom - Visual Odometry Pipeline
% Nikhilesh Alatur, Simon Schaefer
%##########################################################################
clear all; close all; clc;  %#ok<CLALL>

addpath(genpath('functions/'));

%addpath('plotting/'); 
%addpath('features/'); 
%addpath('klt/'); 
%addpath('ransac/'); 
%addpath('debug_new/'); %Debug: Functions 1:1 from exercise solution.

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
    imgs_contop = uint8(zeros(size(img0,1), size(img0,2), last_frame)); % BUGFIX: Convert to uint8, else can't be processed as image.
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
    for i = (bootstrap_frames(2)+1):last_frame    % HACK: Use only every 3rd frame -> More distance between frames -> Smaller depth error.

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

% Detect and extract features in img0.
harris_scores0 = harris(img0, params('patch_size'), params('harris_kappa'));
keypoints0 = selectKeypoints(...
    harris_scores0, params('num_keypoints_init'), params('r_suppression'));
descriptors0 = describeKeypoints(img0, keypoints0, params('r_desciptor'));

% Detect and extract features in img1.
harris_scores1 = harris(img1, params('patch_size'), params('harris_kappa'));
keypoints1 = selectKeypoints(...
    harris_scores1, params('num_keypoints_init'), params('r_suppression'));
descriptors1 = describeKeypoints(img1, keypoints1, params('r_desciptor'));

% Match them!
matches = matchDescriptors(descriptors1, descriptors0, params('match_lambda'));
[~, query_indices, match_indices] = find(matches);
Pq = keypoints1(:, query_indices);
Pdb = keypoints0(:, match_indices);     

% Use the ground truth to set the scale of this bootstrap transformation.
t_norm_base = 0.1294;   % ToDo: Take this from ground truth.

% Estimate the F by ransac, extract the correct R,t and triangulate landmarks.
[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pdb,Pq, K, t_norm_base);  % BUGFIX: Pdb is P1 and Pq is P2 in this function and not the opposite.

%% Plotting - Debugging. 
% plotMatches(flipud(query_kps), flipud(database_kps), img0); 
% plotPointCloud(X); 

%% Assign initial state. 
state = struct; 
state.T = [R_CW t3_CW; [0,0,0,1]];  %BUGFIX: Homogenous transformation matrix has [0,0,0,1] as last row, not [1,1,1,1] 
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

    % Track keypoints with MATLAB's KLT algorithm.
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
    fprintf('KLT number of tracked keypoints: %d\n', nnz(validity));
    
    % Find new camera pose with MATLAB's P3P function.
    imagePoints = fliplr(Pq');
    worldPoints = X(1:3,:);
    worldPoints = worldPoints';
    K_intrinsics = [K(1,1),0,0;...
        0,K(2,2),0;...
        K(1,3),K(2,3),1];   % Cause MATLAB has different convention for K.
    cameraParams = cameraParameters('IntrinsicMatrix', K_intrinsics);
    [worldOrientation,worldLocation,status] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams,...
        'MaxReprojectionError',10, 'MaxNumTrials',2000);
    
    % Convert T_WC->T_CW
    R_WC = worldOrientation;
    t_WC = worldLocation';
    T_WC = [R_WC,t_WC];
    T_WC = [T_WC;[0,0,0,1]];
    T_CW = inv(T_WC);
    R_CW = T_CW(1:3,1:3);
    t3_CW = T_CW(1:3,4);
        
    assert(isequal(size([R_CW t3_CW]), [3,4])); % BUGFIX: The SIZES have to be the same. 
    
    % Triangulate new landmarks, after motion has been estimated.
    centroid_x = mean(Pq(2,:));
    centroid_y = mean(Pq(1,:));
    if size(X,2)<45 %or(abs(centroid_x-320)>32,abs(centroid_y-240)>24)% % TODO: Find optimal indicator for re-triangulation.
        disp("Triangulating new landmarks...")
        % Detect and extract new features in previous frames.
        harris_scores_prev = harris(img_prev, params('patch_size'), params('harris_kappa'));
        keypoints_prev = selectKeypoints(...
            harris_scores_prev, params('num_keypoints_init'), params('r_suppression'));
        descriptors_prev = describeKeypoints(img_prev, keypoints_prev, params('r_desciptor'));
        
        % Detect and extract features in the current frame.
        harris_scores_curr = harris(img, params('patch_size'), params('harris_kappa'));
        keypoints_curr = selectKeypoints(...
            harris_scores_curr, params('num_keypoints_init'), params('r_suppression'));
        descriptors_curr = describeKeypoints(img, keypoints_curr, params('r_desciptor'));
        
        % Match them!
        matches = matchDescriptors(descriptors_curr, descriptors_prev, params('match_lambda'));
        [~, query_indices, match_indices] = find(matches);
        Pq = keypoints_curr(:, query_indices);
        Pdb = keypoints_prev(:, match_indices);     

        % Find the norm of the transform from the last to the current
        % frame. Required for scaling the translation vector from estimateTrafoFund.
        T_CcCp = [R_CW t3_CW; [0,0,0,1]]*inv(T_prev);
        t3_CcCp = T_CcCp(1:3,4);
        
        % Triangulate new landmarks.
        [~, ~, X_new, ~] = estimateTrafoFund(Pdb, Pq, K, norm(t3_CcCp));  % BUGFIX: Use the norm of the relative translation, not w.r.t world frame!
        % OBSERVATION: We have a problem here that there is noise in Z and Y
        % direction. Without it, the difference was exactly 0.5, as GT
        % suggests, but with GT the distance was 1.36. This is okay because
        % we triangulated a completely new pointcloud. But this will
        % inevitably lead to drift and errors in the long term.
        
        % Use the T_CW of the previous frame to bring back the pointcloud to the world frame.
        X = inv(T_prev)*X_new;  
        fprintf('Number of new landmarks: %d\n', size(X_new,2));
    end
    
    % Renew state and add to trajectory.
    state = struct; 
    state.T = [R_CW t3_CW; [0,0,0,1]];  % BUGFIX: Homogenous transformation matrix has [0,0,0,1] as last row, not [1,1,1,1] 
    state.P = Pq; 
    state.Pdb = Pdb; 
    state.X = X; 
    trajectory = [trajectory; state];
    
    % Plotting. 
    plotOverall(img, trajectory); 
    pause(0.01);
end
