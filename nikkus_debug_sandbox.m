close all;
clear all;
clc;
%%
close all;
figure;
img_prev = img0;
img = img1;
pt = [5,11];   %5,7,8
subplot(1,2,1);
imshow(img_prev);
hold on;
plot(Pdb(2, pt), Pdb(1, pt), 'rx', 'Linewidth', 2);
%
subplot(1,2,2);
imshow(img);
hold on;
plot(Pq(2, pt), Pq(1, pt), 'gx', 'Linewidth', 2);

%figure;
plotPointCloud(X(:,pt))


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
for i=1:26
    T_CW = trajectory(i).T;
    T_WC = inv(T_CW);
    pos = T_WC*[0;0;0;1];
    position = [position;(pos(1:3))'];
end
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

%%
