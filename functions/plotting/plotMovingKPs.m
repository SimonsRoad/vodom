function plotMovingKPs(kps, kps_next, img)
imshow(img); 
hold on;
plot((kps_next(2, :))', (kps_next(1, :))', 'rx'); 
plot((kps(2, :))', (kps(1, :))', 'go'); 
end

