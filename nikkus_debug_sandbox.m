close all;
clear all;
clc;
%%
close all;
i = 2;
img_prev = img0;
img = img1;
Pdb = database_kps;
Pq = query_kps;
pt = 11;%[1,4,6,8,9,11];
figure;
imshow(img_prev);
hold on;
plot(Pdb(2, pt), Pdb(1, pt), 'rx', 'Linewidth', 2);
%
figure;
imshow(img);
hold on;
plot(Pq(2, pt), Pq(1, pt), 'gx', 'Linewidth', 2);

%figure;
%plotPointCloud(X(:,pt))


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
for i=2:112
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