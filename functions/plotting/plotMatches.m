function plotMatches(q_kps, db_kps, q_img, db_img)
% Plot features matches in between two images. 
% @param[in]    qm_keypoints    matched query keypoints [2,L]. 
% @param[in]    dbm_keypoints   matched database keypoints [2,L]. 
% @param[in]    q_img           query image. 
% @param[in]    db_img          database image. 
showMatchedFeatures(db_img, q_img, fliplr(db_kps'), fliplr(q_kps'));
end

