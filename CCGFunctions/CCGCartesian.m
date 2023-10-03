function X = CCGCartesian(A,B)
%CZCARTESIAN Summary of this function goes here
%   Detailed explanation goes here
%ultima alteração 04/06
X.G = blkdiag(A.G,B.G);
X.c = [A.c;B.c];
X.A = blkdiag(A.A,B.A); 
X.b = [A.b;B.b];

if isinf(A.type(end)) && isinf(B.type(1))
    X.type = [A.type B.type(2:end)];
    X.idx = [A.idx(1:end-1) A.idx(end)+B.idx(1) B.idx(2:end)];
else
    X.type = [A.type, B.type];
    X.idx = [A.idx, B.idx];
end
end

