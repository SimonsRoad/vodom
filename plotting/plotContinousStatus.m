%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Plotting continous operation - Tracked and newly added features, 
% position and trajectory. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plotContinousStatus(img, S_new, S_tracked, cam_positions, ...
                             num_tracked_keypoints)
% @param[in]    img             current image. 
% @param[in]    S_new           pixel coords of new detected features
%                               (N_new,2)
% @param[in]    S_tracked       pixel coords of tracked features
%                               (N_tracked,2).
% @param[in]    cam_positions   sequence of poses on path (3,L) with L 
%                               the path length. 
% @param[in]    num_tracked_keypoints number of tracked keypoints from 
%                               first to current frame. 
clf('reset');
n_frames = size(num_tracked_keypoints,2); 
n_show = min(20,n_frames); 
% Display image at upper left corner - Add pixel markers for tracked 
% and newly added features. 
subplot(2,4,[1,2]);
imshow(img); 
hold on; 
S_new = S_new'; S_tracked = S_tracked'; 
plot(S_new(1,:),S_new(2,:),'o', 'Color', 'g', 'Linewidth', 2);
plot(S_tracked(1,:),S_tracked(2,:),'x', 'Color', 'r', 'Linewidth', 2);
hold off; 
title('Current Image (new = green, tracked = red)');
% Display number of tracked keypoints over last frames.
subplot(2,4,5);
plot(-n_show+1:0,num_tracked_keypoints(end-n_show+1:end)); 
axis equal;
xlabel('Frame index (0 = current)');
ylabel('# tracked keypoints');
title('# previously tracked keypoints')
% Display trajectory. 
subplot(2,4,6);
plot(cam_positions(1,:),cam_positions(3,:)); 
axis equal;
xlabel('x');
ylabel('z');
title('Full Trajectory')
% Reset window position and size. 
%set(gcf,'units','points','position',[100,100,600,400])
end