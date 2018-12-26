function plotMovingKPs(kps, kps_next, img)
% Plot moving feature positions in image frame. 
x_from = kps(1, :);
x_to = kps_next(1, :);
y_from = kps(2, :);
y_to = kps_next(2, :);
imshow(img); hold on;
plot(kps(2, :), kps(1, :), 'rx', 'Linewidth', 2); 
plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 3);
end

