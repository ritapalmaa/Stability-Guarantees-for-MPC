function Z = CCGReduction(Z,a,d)
% CCGReduction - Function that calculates the reduction around a 
% Constrained Convex Generator Z considering the value for reduction.
%  
% Syntax:  
%    Z = CCGReduction(Z,a,d)
%
% Inputs:
%    Z - Constrained Convex Generator
%    a - value to be reduced around Z
%    d - The distance from the center to the furthest point in Z 
%        in each direction
%
% Outputs:
%    Z - Constrained Convex Generator reduced
%
% Example: 
%    m = 2;
%    Z.G = rand(2,m);
%    Z.c = zeros(2,1);
%    Z.A = zeros(0,m);
%    Z.b = zeros(0,1);
%    Z.idx = [1 1];
%    Z.type = [2 inf];
%    a = 0.1;
%    d = {norm(Z.G(1,:),Inf); norm(Z.G(2,:),Inf)};
%    Zred = CCGReduction(Z,a,d);
%    [F,p] = compileCCG(Z);
%    [Fred,pred] = compileCCG(Zred);
%    figure;
%    plot(F,p);hold on;
%    plot(Fred,pred,'g');
%
% Other m-files required: none.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

for i = 1 : size(Z.G,1)
    for j = 1:size(Z.G,2)
            Z.G(i,j) = Z.G(i,j) * (1-a/d{i});
    end
end
end

