function [P, Pdb] = harrisMatching(img, img_db, ...
                    num_kps,r_filter,kappa,r_sup,r_desc,lambda)
% Detect and extract features in img0.
scores_prev = harris(img_db, r_filter, kappa);
kp_prev = selectKeypoints(scores_prev, num_kps, r_sup);

%status1 = abs(kp_prev(2,:)-620) > 100;
%status2 = kp_prev(1,:)-188 > 0;
%kp_prev = kp_prev(:,status1 & status2);  

desc_prev = describeKeypoints(img_db, kp_prev, r_desc);
% Detect and extract features in img1.
harris_scores = harris(img, r_filter, kappa);
kp = selectKeypoints(harris_scores, num_kps, r_sup);

%status1 = abs(kp(2,:)-620) > 100;
%status2 = kp(1,:)-188 > 0;
%kp = kp(:,status1 & status2);  

desc = describeKeypoints(img, kp, r_desc);
% Match them!
matches = matchDescriptors(desc, desc_prev, lambda);
[~, query_indices, match_indices] = find(matches);
P = kp(:, query_indices);
Pdb = kp_prev(:, match_indices); 
end