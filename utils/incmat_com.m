function a = incmat_com(D,R)
%INCMAT_COM computes the components of the graph's incident matrix
%Inputs: node distance D, max communication threshold R; Output: matrix component a
if D <= R
    w = (D / R)^2;   % polynomial bump (smooth, bounded)
    a = (1 - w)^3;   % in [0,1]
end