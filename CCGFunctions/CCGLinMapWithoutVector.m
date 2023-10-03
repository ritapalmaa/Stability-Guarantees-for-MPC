function Y = CCGLinMapWithoutVector(A,X)
%CZLINMAP Summary of this function goes here
%   Detailed explanation goes here
Y.G = A * X.G;
Y.c = A * X.c;
Y.A = X.A;
Y.b = X.b;
Y.type = X.type;
Y.idx = X.idx;
end

