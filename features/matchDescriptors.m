%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vOdom - Visual Odometry Pipeline
% Nikhilesh Alaturn, Simon Schaefer
% Match descriptors using sum of squared distances (SSD). Filter out 
% non-unique matches. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function C = matchDescriptors(D1, D2)
% @param[in]    D1      set of descriptors in first image (N1,M). 
% @param[in]    D2      set of descriptors in second image (N2,M). 
% which M as length of descriptor vector (has to be equal !). 
assert(size(D1,2) == size(D2,2)); 
C = matchFeatures(D1,D2, 'Metric','SSD', 'Unique',true); 
end