function plotOverall(img, trajectory)
clf('reset');
n = size(trajectory, 1); 
if n < 2
    return
end
% Display image at upper left corner - Add pixel markers for tracked 
% and newly added features. 
subplot(2,3,1);
P_prev = trajectory(end).Pdb; 
P_new  = trajectory(end).P; 
plotMovingKPs(P_prev, P_new, img); 
hold off; 
title('Current Image (new = green, tracked = red)');
% Display number of tracked keypoints over last frames.
subplot(2,3,4);
num_keypoints = zeros(1,n); 
num_hist_kps = 10; 
for i = 1:n
    num_keypoints(i) = size(trajectory(i).P, 2); 
end
if n > num_hist_kps
    plot(n-num_hist_kps:n, num_keypoints(end-num_hist_kps:end)); 
else
    plot(1:n, num_keypoints);
end
xlabel('Frame index');
ylabel('# tracked keypoints');
axis([n-num_hist_kps n 0 300])
title('# prev. tracked keypoints')
% Display trajectory. 
subplot(2,3,[2,3,5,6]);
positions = zeros(2,n); 
for i = 1:n
   trajectory(i).T
   positions(:,i) = trajectory(i).T(1:2,4);  
end
plot(positions(1,:),positions(2,:)); hold on; 
plot(trajectory(end).X(1,:),trajectory(end).X(2,:), '*'); 
xlabel('x');
ylabel('y');
axis([-5.0 5.0 -5.0 5.0])
title('Full Trajectory')
% Reset window position and size. 
%set(gcf,'units','points','position',[100,100,600,400])
end