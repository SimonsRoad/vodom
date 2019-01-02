function plotPointCloud(cloud)
% Plot triangulated feature point cloud. 
% @param[in]    cloud       triangulated feature world points [3,L]. 
figure; 
xs = cloud(1,:); 
ys = cloud(2,:); 
zs = cloud(3,:);

scatter3(xs',ys',zs');
xlabel('X');
ylabel('Y');
zlabel('Z');
end

