close all;
clear all;
clc;

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