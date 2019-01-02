function [query_kps, database_kps] = matches2kps(matches, query_kps, db_kps)
[~, query_indices, match_indices] = find(matches);
query_kps = query_kps(:, query_indices);
database_kps = db_kps(:, match_indices);     
end