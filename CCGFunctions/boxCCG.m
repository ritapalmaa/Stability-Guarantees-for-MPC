function Zbox = boxCCG(Z)
%boxCCG - Function that calculates the interval that overbounds the
% Constrained Convex Generator.
%
% Syntax:  
%   Zbox = boxCCG(Z)
%
% Inputs:
%    Z - Constrained Convex Generator
%
% Outputs:
%    Zbox - Constrained Convex Generator matching an Interval
%
% Example: 
%    m = 10;
%    X.G = rand(2,m);
%    X.c = zeros(2,1);
%    X.A = zeros(0,m);
%    X.b = zeros(0,1);
%    X.idx = [5 5];
%    X.type = [2 inf];
%    Zbox = boxCCG(Z);
%    [Fx,px] = compileZonotope(X);
%    [Fz,pz] = compileZonotope(Z);
%    plot(Fz,pz,'b');hold on;
%    plot(Fx,px);
%
% Other m-files required: compileCCG.m, CCGOverbound.m.
% Subfunctions: none
% MAT-files required: none
%
% See also: none.

%------------- BEGIN CODE --------------

n = size(Z.G, 1);

limits = zeros(n,2);

[F,p] = compileCCG(Z);

v = sdpvar(n,1);

bounds = optimizer(F, v'*p,sdpsettings('solver','gurobi,mosek','cachesolvers',1),v,p);

for i = 1:n
    limits(i,:) = [e(i,n)'*bounds(e(i,n)),e(i,n)'*bounds(-e(i,n))];
end

Zbox = CCGOverbound(getSimpleSets('ball',n,0.5*(limits(:,2)-limits(:,1)),inf));
Zbox.c = 0.5*(limits(:,2)+limits(:,1));

end

