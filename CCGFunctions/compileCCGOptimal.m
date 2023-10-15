function [F,p,A,b,G,c] = compileCCGOptimal(Z)
% compileCCGOptimal - Function returning a Yalmip constraint set representing the
% Constrained Convex Generator with the parameters defined by sdpvars.
%   
%   Z = {z = G*xi+c: A*xi = b, xi_j \in C_j, forall j}
% 
% Syntax:  
%    [F,p] = compileCCG(Z)
%
% Inputs:
%    Z - Constrained Convex Generator
%
% Outputs:
%    F - Yalmip constraint set
%    p - sdpvar representing a point in Z.
%    A - sdpvar matrix linear constraint Constrained Convex Generator
%    b - sdpvar vector linear constraint Constrained Convex Generator
%    G - sdpvar generator matrix Constrained Convex Generator
%    c - sdpvar centre Constrained Convex Generator
% 
% Reminder - This function serves the sole purpose of creating an optimizer
% featuring a terminal set that varies with each run. Running this function
% within a singularity environment is not feasible.
% Other m-files required: none.
% Subfunctions: none
% MAT-files required: none
%
% See also: none.


%------------- BEGIN CODE --------------

n = size(Z.G,1);

p = sdpvar(n,1);

xi = sdpvar(size(Z.G,2),1);

A = sdpvar(size(Z.A,1),size(Z.A,2),'full');
b = sdpvar(size(Z.b,1),size(Z.b,2),'full');
G = sdpvar(size(Z.G,1),size(Z.G,2),'full');
c = sdpvar(size(Z.c,1),size(Z.c,2),'full');

pointer = 0;
if isempty(Z.A)
    F = p == G * xi + c;
else
    F = [A * xi == b, p == G * xi + c];
end

for i = 1:length(Z.idx)
    F = [F, norm(xi(pointer + (1:Z.idx(i)),1), Z.type(i)) <= 1];
    pointer = pointer + Z.idx(i);
end
end