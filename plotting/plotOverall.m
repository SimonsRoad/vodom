function plotOverall(img, trajectory)
clf('reset');
n = size(trajectory, 1); 
if n < 2
    return
end
% Display image at upper left corner - Add pixel markers for tracked 
% and newly added features. 
subplot(2,4,[1,2]);
P_prev = trajectory(end).Pdb; 
P_new  = trajectory(end).P; 
plotMovingKPs(P_prev, P_new, img); 
hold off; 
title('Current Image (new = green, tracked = red)');
% Display number of tracked keypoints over last frames.
subplot(2,4,5);
num_keypoints = zeros(1,n); 
for i = 1:n
    num_keypoints(n) = size(trajectory(i).P, 2); 
end
plot(1:n, num_keypoints); 
axis equal;
xlabel('Frame index');
ylabel('# tracked keypoints');
title('# prev. tracked keypoints')
% Display trajectory. 
subplot(2,4,6);
positions = zeros(2,n); 
for i = 1:n
   positions(:,i) = trajectory(i).T(4,1:2);  
end
plot(positions(1,:),positions(2,:)); 
axis equal;
xlabel('x');
ylabel('z');
title('Full Trajectory')
% Reset window position and size. 
%set(gcf,'units','points','position',[100,100,600,400])
end