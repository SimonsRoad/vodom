function matches = matching(query_descriptors, database_descriptors, params)
% Match descriptor values to pairs using SSD measure. 
% @param[in]    query_descriptors   query descriptors [M,Q]. 
% param[in]     database_desciptors database descriptors [M,D]. 
% @param[out]   matches             matched database descriptors [1,Q]. 
% matches(i) will be zero if there is no database descriptor with an 
% SSD < lambda * min(SSD). 
lambda = params('match_lambda'); 
[dists,matches] = pdist2(double(database_descriptors)', ...
    double(query_descriptors)', 'euclidean', 'Smallest', 1);
sorted_dists = sort(dists);
sorted_dists = sorted_dists(sorted_dists~=0);
min_non_zero_dist = sorted_dists(1);
matches(dists >= lambda * min_non_zero_dist) = 0;
% Remove double matches
unique_matches = zeros(size(matches));
[~,unique_match_idxs,~] = unique(matches, 'stable');
unique_matches(unique_match_idxs) = matches(unique_match_idxs);
matches = unique_matches;
end

