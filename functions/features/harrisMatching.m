function [P, Pdb] = harrisMatching(img, img_db, r_desc, match_ratio)
% Detect and extract features in img0.
harris_prev = detectHarrisFeatures(img_db); 
[desc_prev,valid_prev] = extractFeatures(img_db,harris_prev, ...
    'Method','Block', 'BlockSize', r_desc);
% Detect and extract features in img1.
harris = detectHarrisFeatures(img); 
[desc,valid] = extractFeatures(img,harris, ...
    'Method','Block', 'BlockSize', r_desc);
% Match them!
matches = matchFeatures(desc_prev, desc, 'MaxRatio', match_ratio);
Pdb = double(flipud(valid_prev(matches(:,1),:).Location'));
P = double(flipud(valid(matches(:,2),:).Location'));
end