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
% [P1, dc1] = features(img0, params, false);   % P1, P2 have form [v;u]
% [P2, dc2] = features(img1, params, false); % P1<->Pdb ; P2<->Pq
% matches = matching(dc1, dc2, params);
% [Pq, Pdb] = matches2kps(matches, P2, P1);   
% %%
%[R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan); % BUG?: R_CW is nowhere close to identity.
% ##########################DEBUG START: BOOTSTRAP#######################
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

% M = estimatePoseDLT(fliplr(query_kps'), (X(1:3,:))', K)
% NOTE: I have verified, that Until here, the algorithm runs correctly, the
% matched points are good and the triangulated landmarks (X) make sense as
% well. Only problem is with triangulated landmarks, where the matched
% features were too close together -> Big depth error.
% However, DLT is returning wrong results. When feeding X and query_kpts
% to DLT, the output doesn't resemble what we got from bootstrapping!
% ##########################DEBUG-END: BOOTSTRAP#######################

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

%% Testing.
% clc
% disp("Starting bootstrapping initialization ..."); 
% [P1, dc1] = features(imgs_bootstrap(1:480,:), params, false);
% [P2, dc2] = features(imgs_bootstrap(480:end,:), params, false);
% matches = 1:size(P1,2); 
% [Pq, Pdb] = matches2kps(matches, P2, P1);
% [R_CW, t3_CW, X, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan)
% 
% state_prev = trajectory(end); 
% T = state_prev.T; 
% P = state_prev.P; 
% X = state_prev.X; 
% 
% projected = projectPoints(X, K); 
% for i = 1:size(P,2)
%    disp("----------------"); 
%    disp(projected(:,i)); 
%    disp(P(:,i)); 
% end

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
    if false %size(P_prev, 2) < 20%params('klt_min_num_kps')   % DEBUG: Don't use this for now.
        disp("Reinitializing ... "); 
        % BUG: The world frame gets reset, when do it like this!
%         [P1, dc1] = features(img_prev, params, true);
%         [P2, dc2] = features(img, params, true);
%         matches = matching(dc1, dc2, params);
%         [Pq, Pdb] = matches2kps(matches, P2, P1);
%         [R_CW, t3_CW, X, ~] = estimateTrafoFund(Pq, Pdb, K, t3norm);

        % ####################DEBUG START: RE-INIT#############################
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
        query_kps = keypoints_2(:, query_indices);
        database_kps = keypoints(:, match_indices); 
        
        Pq = query_kps;
        Pdb = database_kps;
        [R_CW, t3_CW, X, ~] = estimateTrafoFund(Pq, Pdb, K, t3norm);
        % ####################DEBUG END: RE-INIT#############################
        
        fprintf('Reinitialization number of matches: %d\n', size(matches,2));
    % Otherwise apply KLT for every keypoint. 
    else
%         disp("KL Tracking ..."); 
%         [P, keep] = trackKLT(img_prev, img, P_prev, params); % BUG: KLT doesn' track well. The locations don't make sens and inconsitent what patch harris returns.
%         % Estimate transformation from previous to current frame. 
%         Pq = P(:,keep); 
%         Pdb = P_prev(:,keep); 
%         Xdb = X_prev(:,keep); 
%         %[R_CW, t3_CW, ~] = estimateTrafoP3P(Pq, Xdb, K, t3norm, params); % BUG: Something's wrong! Output from p3p is nowhere close to a rotation matrix!
% %         M = dlt(fliplr(Pq'), (Xdb(1:3,:))', K); % Because P3P is failing, just do DLT with the tracked keypoints and the landmark pointcloud.
% %         R_CW = M(:,1:3);
% %         t3_CW = M(:,4);
        %##################DEBUG-START: KLT###############################
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

        [R_CcCp, t3_CcCp, ~, t3norm] = estimateTrafoFund(Pq, Pdb, K, nan);

        %[R_CW, t3_CW, ~, ~] = estimateTrafoFund(Pq, Pdb, K, nan);
        
        %##################DEBUG-START: KLT###############################
        X = Xdb; %BUGFIX: Xdb, instead of X_prev  
        fprintf('KLT number of tracked keypoints: %d\n', nnz(keep));
    end
    assert(isequal(size([R_CW t3_CW]), [3,4])); 
    %#########################DEBUG-START: Triangulate new landmarks############
    % If too little features kept: Triangulate new landmarks, by using the
    % just computed R_CW, t_CW.
    if false%nnz(keep)<15
        disp("Triangulating new landmarks...")
        % 1.) Detect and match features between img, img_prev.
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
        query_kps = keypoints_2(:, query_indices);
        database_kps = keypoints(:, match_indices); 
        
        % 2.) Triangulate new landmarks with M_prev, M_curr       
        X_new = linearTriangulation([flipud(database_kps);ones(1,length(database_kps))],...
            [flipud(query_kps);ones(1,length(query_kps))],...
            K*state(end).T(1:3,:),K*[R_CW t3_CW]);

        % 3.) Overwrite landmarks and database.      
        Pdb = database_kps;
        Pq = query_kps;
        X = X_new;
    end    
    %#########################DEBUG-END: Triangulate new landmarks############
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
