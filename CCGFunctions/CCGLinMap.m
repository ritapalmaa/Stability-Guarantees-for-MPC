function Y = CCGLinMap(A,X,t)
% CCGLinMap - Function that calculates the linear map AX + t 
%  
% Syntax:  
%    Y = CCGLinMap(A,X,t)
%
% Inputs:
%    A - A matrix 
%    X - Constrained Convex Generator
%    t - translation vector
%
% Outputs:
%    Y - Constrained Convex Generator that Y = {y = A*x + t: x \in X}
%
% Example: 
%    m = 10;
%    X.G = rand(2,m);
%    X.c = zeros(2,1);
%    X.A = zeros(0,m);
%    X.b = zeros(0,1);
%    X.idx = [5 5];
%    X.type = [2 inf];
%    Y = CCGLinMap(2*eye(2),X,zeros(2,1));
%    [Fx,px] = compileCCG(X);
%    [Fy,py] = compileCCG(Y);
%    plot(Fy,py,'b');hold on;
%    plot(Fx,px);
%
% Other m-files required: none.
% Subfunctions: none
% MAT-files required: none
%
% See also: none.
 
% Copyright 2023 Daniel Silvestre
% This file is part of ReachTool.
%
% ReachTool is free software: you can redistribute it and/or modify 
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ReachTool is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with ReachTool.  If not, see <https://www.gnu.org/licenses/>.

 
%------------- BEGIN CODE --------------

Y.G = A * X.G;
Y.c = A * X.c + t;
Y.A = X.A;
Y.b = X.b;
Y.type = X.type;
Y.idx = X.idx;

if isfield(X,'sidx')
    Y.sidx = X.sidx;
    Y.weights = X.weights;
    Y.freeTerm = X.freeTerm;
end
end
