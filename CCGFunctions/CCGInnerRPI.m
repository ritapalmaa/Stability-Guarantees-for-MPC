function RPI = CCGInnerRPI(A, U, N)
%CCGInnerRPI - Function that calculates the inner Robust Positively 
% Invariant set considering the closed-loop system and the disturbances.
% 
% Syntax:  
%    RPI = CCGInnerRPI(A, U, N)
%
% Inputs:
%    A - closed-loop matrix
%    U - Constrained Convex Generator of disturbances
%    N - Horizon
%
% Outputs:
%    RPI - Constrained Convex Generator 
%
% Example: 
%    n = 2;
%    m = 1;
%    U.G = [zeros(m,m); 1.5*rand(m)];
%    U.c = zeros(n,1);
%    U.A = zeros(0,m);
%    U.b = zeros(0,1);
%    U.idx = m;
%    U.type = Inf;
%    A = rand(n);
%    N = 4;
%    RPI = CCGInnerRPI(A, U, N);
%    figure;
%    [F,p] = compileCCG(RPI);
%    [Fu,pu] = compileCCG(U);
%    plot(F,p);hold on;
%    plot(Fu,pu);
%
% Other m-files required: boxCCG.m.
% Subfunctions: none
% MAT-files required: none
%
% See also: none.

%------------- BEGIN CODE --------------

% Get a box set since currently it only supports lp balls
if ~isempty(U.A) 
    U = boxCCG(U);
end

% Value equivalent to infinity
Kinf = 1000;

% Numerically calculate overbound for the remainder of the expression (NEED
% TO BE IMPROVED)
rest = zeros(length(A)); 
for i = N+1:Kinf
    rest = rest + A^i;
end

n = size(A,1);

Rest.G = rest*diag(max(U.G,[],2));
Rest.c = zeros(n,1);
Rest.A = zeros(0,n);
Rest.b = zeros(0,1);
Rest.type = 2; 
Rest.idx = n;

% Explicitly calculate the RPI set
RPI = U;
for i = 1:N
    RPI = CCGMinkowskiSum(RPI, CCGLinMap(A^i,U,zeros(n,1)));
end

RPI = CCGMinkowskiSum(RPI, Rest);
end