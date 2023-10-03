function X = AddZeros(X, Xmax)
% addzeros function takes a CCG and fills in the blank spaces with zeros
% to make it the same size as the larger CCG received.
    if size(X.G) == size(Xmax)
        return
    else
        X.G = [X.G zeros(size(X.G,1),length(Xmax.G)-length(X.G))];
        X.A = padarray(X.A,[size(Xmax.A,1)-size(X.A,1) size(Xmax.A,2)-size(X.A,2)],0,'post');
        X.b = [X.b; zeros(length(Xmax.b)-length(X.b),1)];     
    end
end