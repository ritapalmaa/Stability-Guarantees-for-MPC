function Z = CCGCartesian(A,B)
% CCGCartesian - Function that calculates the cartesian product A x B
% both Constrained Convex Generators.
%  
% Syntax:  
%    Z = CCGCartesian(A,B)
%
% Inputs:
%    A - Constrained Convex Generator 
%    B - Constrained Convex Generator
%
% Outputs:
%    Z - Constrained Convex Generator that Z = {z = (A.G 0; 0 B.G)*x + (A.c; B.c) : (A.A 0; 0 B.A)*xi = (A.b; B.b), xi_j \in C_j, forall j}
%
% Example: 
%    m = 10;
%    A.G = rand(2,m);
%    A.c = zeros(2,1);
%    A.A = zeros(0,m);
%    A.b = zeros(0,1);
%    A.idx = [5 5];
%    A.type = [2 inf];
%    B = A;
%    B.G = rand(2,m);
%    Z = CCGCartesian(A,B);
%    [Fa,pa] = compileCCG(A);
%    [Fb,pb] = compileCCG(B);
%    [Fz,pz] = compileCCG(Z);
%    plot(Fa,pa);hold on;
%    plot(Fb,pb,'g');
%    plot(Fz,pz,'b');
%
% Other m-files required: none.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

Z.G = blkdiag(A.G,B.G);
Z.c = [A.c;B.c];
Z.A = blkdiag(A.A,B.A); 
Z.b = [A.b;B.b];

if isinf(A.type(end)) && isinf(B.type(1))
    Z.type = [A.type B.type(2:end)];
    Z.idx = [A.idx(1:end-1) A.idx(end)+B.idx(1) B.idx(2:end)];
else
    Z.type = [A.type, B.type];
    Z.idx = [A.idx, B.idx];
end
end

