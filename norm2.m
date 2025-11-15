function n = norm2(z)
%NORM2 computes the Euclidean norm of a vector
%Input: vector z; Output: 2-norm of z
n = sqrt(sum(z.^2));
end