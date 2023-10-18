function nextX = SetReduction(maxError,X,nx,nu)
% SetReduction - Function that calculates a reduced set nextX based on X
%  
% Syntax:  
%    nextX = SetReduction(maxError,X,nx,nu)
%
% Inputs:
%    maxError - solver's error
%    X - Constrained Convex Generator
%    nx - state dimensions
%    nu - input dimensions
%
% Outputs:
%    nextX - Constrained Convex Generator reduced
%
% Other m-files required: CCGLinMap.m, boxCCG.m, CCGReduction.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Define the set of error in xi
E.G = maxError*eye(nx); E.c = zeros(nx,1);
E.A = zeros(0,nx); E.b = zeros(0,1); 
E.type = Inf; E.idx = nx;

% Calculate the impact of the error in the generators
reduction_set = CCGLinMap(X.G',E,zeros(size(X.G,2),1));
red_box = boxCCG(reduction_set);
red_value = max(max(red_box.G));

% Calculate the maximum distance from the center to the furthest point
rad = cell(nx,1); idx = 1;
for i = 1 : nu : nx
    for j = 0 : nu-1
        rad{i + j} = norm(X.G(i:idx*nu,:),Inf);
    end
    idx = idx +1;
end

% Calculate and compile the set reduced
nextX = CCGReduction(X,red_value,rad);
end

