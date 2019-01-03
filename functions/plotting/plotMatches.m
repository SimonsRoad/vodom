function plotMatches(qm_keypoints, dbm_keypoints, q_img)
% Plot features matches in between two images. 
% @param[in]    qm_keypoints    matched query keypoints [2,L]. 
% @param[in]    dbm_keypoints   matched database keypoints [2,L]. 
% @param[in]    q_img           query image. 
P1 = qm_keypoints; 
P2 = dbm_keypoints; 
x_from = P1(2, :)';
x_to = P2(2, :)';
y_from = P1(1, :)';
y_to = P2(1, :)';
imshow(q_img); hold on;
plot((P1(2, :))', (P1(1, :))', 'rx'); 
plot((P2(2, :))', (P2(1, :))', 'go'); 
plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
hold off; 
end

