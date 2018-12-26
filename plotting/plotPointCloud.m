function plotPointCloud(cloud)
% Plot triangulated feature point cloud. 
% @param[in]    cloud       triangulated feature world points [3,L]. 
figure; 
xs = cloud(1,:); 
ys = cloud(2,:); 
plot(ys, xs, '*');
end

