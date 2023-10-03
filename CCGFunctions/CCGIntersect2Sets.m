function nextX = CCGIntersect2Sets(X, Y)
%CZUPDATE Summary of this function goes here
%   Detailed explanation goes here

nextX.G = [X.G zeros(size(X.G,1) , size(Y.G,2))];
nextX.c = X.c;
if isempty(Y.A) && ~isempty(X.A)
    nextX.A = [X.A zeros(size(X.A,1),size(Y.G,1));X.G -Y.G];
else
    nextX.A = [blkdiag(X.A,Y.A);X.G -Y.G];
end

nextX.b = [X.b;Y.b;Y.c - X.c];

if isinf(X.type(end)) && isinf(Y.type(1))
    nextX.type = [X.type Y.type(2:end)];
    nextX.idx = [X.idx(1:end-1) X.idx(end)+Y.idx(1) Y.idx(2:end)];
else
    nextX.type = [X.type, Y.type];
    nextX.idx = [X.idx, Y.idx];
end

end

