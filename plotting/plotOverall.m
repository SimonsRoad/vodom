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
title('Current Image (red=tracked)');
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
%positions = zeros(2,n); 
positions = [];
for i=1:n
    T_CW = trajectory(i).T;
    T_WC = inv(T_CW);
    pos = T_WC*[0;0;0;1];
    positions = [positions;(pos(1:3))'];
end

plot(positions(:,1),positions(:,3)); % NOTE: We are interested in X-Z (topview) and not in X-Y!
hold on; 
plot((trajectory(end).X(1,:))',(trajectory(end).X(3,:))', '*'); 
xlabel('X_W');
ylabel('Z_W');
axis equal;
%axis([-10 200.0 -3.0 3.0])
title('Full Trajectory')

% Reset window position and size. 
%set(gcf,'units','points','position',[100,100,600,400])
end