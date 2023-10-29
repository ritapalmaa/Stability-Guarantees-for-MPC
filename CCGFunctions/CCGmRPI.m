function RPI = CCGmRPI(A, U, N)
%CCGInnerRPI - Function that calculates the inner Robust Positively 
% Invariant set considering the closed-loop system and the disturbances.
% 
% Syntax:  
%    RPI = CCGmRPI(A, U, N)
%
% Inputs:
%    A - closed-loop matrix
%    U - Constrained Convex Generator of disturbances
%    N - Prediction horizon
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

% Initialization
n = size(A,1);

% Get a box set since currently it only supports lp balls
if ~isempty(U.A) 
    U = boxCCG(U);
end

% Numerically calculate overbound for the remainder of the expression
rest = inv(eye(size(A))-A);
for i = 0:N
    rest = rest-A^i;
end
Rest = CCGLinMap(rest,U,zeros(n,1));

% Explicitly calculate the RPI set
RPI = U; 
for i = 1:N
    RPI = CCGMinkowskiSum(RPI, CCGLinMap(A^i,U,zeros(n,1)));
end

RPI = CCGMinkowskiSum(RPI, Rest);
end