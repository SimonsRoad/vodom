function plotMovingKPs(kps, kps_next, img)
% Plot moving feature positions in image frame. 
%!!!!!!!!!!!!!!!!!!! KEYPOINTS HAVE X,Y FLIPPED!!!!!!!!!!!!!!!!!!!!!!!!!!!
% x_from = kps(1, :);
% x_to = kps_next(1, :);
% y_from = kps(2, :);
% y_to = kps_next(2, :);
imshow(img); hold on;
plot((kps_next(2, :))', (kps_next(1, :))', 'rx'); 
%plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
end

