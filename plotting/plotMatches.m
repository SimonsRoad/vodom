function plotMatches(qm_keypoints, dbm_keypoints, q_img)
% Plot features matches in between two images. 
% @param[in]    qm_keypoints    matched query keypoints [2,L]. 
% @param[in]    dbm_keypoints   matched database keypoints [2,L]. 
% @param[in]    q_img           query image. 
x_from = qm_keypoints(1, :);
x_to = dbm_keypoints(1, :);
y_from = qm_keypoints(2, :);
y_to = dbm_keypoints(2, :);
figure; 
imshow(q_img); hold on;
plot(qm_keypoints(1, :), qm_keypoints(2, :), 'rx', 'Linewidth', 2);
plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
end

