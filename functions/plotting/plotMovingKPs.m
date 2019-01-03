function plotMovingKPs(kps, img)
imshow(img); 
hold on;
plot((kps(2, :))', (kps(1, :))', 'go'); 
end

