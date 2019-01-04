close all;
clear all;
clc;
%%
close all;
%img_prev = img0;
%img = img1;
pt = [20:30];   %5,7,8
figure;
subplot(1,2,1);
imshow(img0);
hold on;
plot(Pdb(2, pt), Pdb(1, pt), 'rx', 'Linewidth', 2);
%
subplot(1,2,2);
imshow(img1);
hold on;
plot(P(2, pt), P(1, pt), 'gx', 'Linewidth', 2);

%%
figure; ax = axes;
showMatchedFeatures(img_prev,img,fliplr((Pdb(:,pt))'),fliplr((Pq(:,pt))'),'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

%% 
for i=1:100
       R = [1     0     0;
         0     0    -1;
         0     1     0];

grid on;
axis equal;
axis manual;

cam = plotCamera('Location',[sin(i)*10 0 20],'Orientation',R,'Opacity',0);

xlim([-15,20]);
ylim([-15,20]);
zlim([15,25]);
    
    
    
    pause(0.1);
 
    
end

%%
position = [];
for i=1:52
    T_CW = trajectory(i).T;
    T_WC = inv(T_CW);
    pos = T_WC*[0;0;0;1];
    position = [position;(pos(1:3))'];
end

close all;
plot(position(:,1),position(:,3))
hold on;
xlabel('X');
ylabel('Z')
axis equal;

%%
scatter(position(:,1),position(:,2))
hold on
xlabel('X')
ylabel('Z')

%%
I_R = imread('000000.png');
%I_R = imresize(I_R, 0.25);
keypoints_rc = load('keypoints.txt');% / 4;
keypoints_flip = flipud(keypoints_rc(1:50, :)');
figure(4);
imshow(I_R);
hold on;
plot(keypoints_flip(1, :), keypoints_flip(2, :), 'rx');
hold off;
I_prev = I_R;
pause(0.1);

%% MASTER Plot
close all;

positions = [];
landmarks_curr = [];
figure;
for i=1:70
    clf('reset');
    
    % Get Current 3D Landmarks
    X_curr = trajectory(i).X;

    % Get current camera pose.
    T_CW = trajectory(i).T;
    T_WC = inv(T_CW);  
    
    % Get trajectory so far    
    pos = T_WC*[0;0;0;1];
    positions = [positions;(pos(1:3))'];
    
    % Plot stuff.
    scatter3((X_curr(1,:))',(X_curr(2,:))',(X_curr(3,:))','rx');
    hold on;
    cam = plotCamera('Location',(pos(1:3))','Orientation',T_WC(1:3,1:3),'Opacity',0,'Color','g','Size',0.5);
    hold on;
    plot3(positions(:,1),positions(:,2),positions(:,3),'b')
    hold on;
    set(gca,'CameraUpVector',[0 -1 0]); % Make the y-axis point down.
    xlabel('X');
    ylabel('Y');
    zlabel('Z');    
    axis ([-10,10,-10,10,-10,10]);
    axis manual;

    
    pause(0.1);
end


