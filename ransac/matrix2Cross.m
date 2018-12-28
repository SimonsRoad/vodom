function x = matrix2Cross(M)
%assert(abs(M(3,2)+M(2,3)) < 1e-4); 
%assert(abs(M(3,1)+M(1,3)) < 1e-4); 
%assert(abs(M(2,1)+M(1,2)) < 1e-4); 
x = zeros(3,1); 
x(1) = M(2,1); 
x(2) = M(1,3); 
x(3) = M(3,2); 
end
