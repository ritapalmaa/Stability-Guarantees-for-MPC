function X = AddZeros(X, Xmax)
% AddZeros - Function that ajust the dimensions of a given set X to match
% Xmax by adding zeros to the set's matrices while maintaining the integrity 
% of the set's constraints
%
% Syntax:  
%    X = AddZeros(X, Xmax)
%
% Inputs:
%    X - Constrained Convex Generator
%    Xmax - Constrained Convex Generator
%
% Outputs:
%    X - Constrained Convex Generator
%
% Other m-files required: none.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

if size(X.G) == size(Xmax)
    return
else
    X.G = [X.G zeros(size(X.G,1),length(Xmax.G)-length(X.G))];
    X.A = padarray(X.A,[size(Xmax.A,1)-size(X.A,1) size(Xmax.A,2)-size(X.A,2)],0,'post');
    X.b = [X.b; zeros(length(Xmax.b)-length(X.b),1)];     
end
end