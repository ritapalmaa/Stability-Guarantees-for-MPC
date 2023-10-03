function Z = CCGMinkowskiSum(X,Y)
%CZMINKOWSKISUM Summary of this function goes here
%   Detailed explanation goes here

Z.G = [X.G Y.G];
Z.c = X.c + Y.c;
if isempty(Y.A) && ~isempty(X.A)
    Z.A = [X.A, zeros(size(X.A,1),size(Y.G,2))];
else
    Z.A = blkdiag(X.A, Y.A);
end
Z.b = [X.b; Y.b];

if isinf(X.type(end)) && isinf(Y.type(1))
    Z.type = [X.type Y.type(2:end)];
    Z.idx = [X.idx(1:end-1) X.idx(end)+Y.idx(1) Y.idx(2:end)];
else
    Z.type = [X.type, Y.type];
    Z.idx = [X.idx, Y.idx];
end

end

