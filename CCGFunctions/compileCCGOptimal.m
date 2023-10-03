function [F,p,A,b,G,c] = compileCCGOptimal(Z)
%COMPILECNB Returns the constraint set associated with the Constrained Norm
%Ball Z.
%   p is the variable
%
%   xi is a auxiliary variable
% 
%   F = [ ||xi1||_inf <= 1, 
%         ||xi2||_2   <= 1, 
%          A * xi == b, 
%          p == G * xi + c];

n = size(Z.G,1);

p = sdpvar(n,1);

xi = sdpvar(size(Z.G,2),1);

A = sdpvar(size(Z.A,1),size(Z.A,2),'full');
b = sdpvar(size(Z.b,1),size(Z.b,2),'full');
% A = Z.A; b = Z.b;
G = sdpvar(size(Z.G,1),size(Z.G,2),'full');
c = sdpvar(size(Z.c,1),size(Z.c,2),'full');

pointer = 0;
if isempty(Z.A)
    F = p == G * xi + c;
%     F = p == G * xi + Z.c;
else
    F = [A * xi == b, p == G * xi + c];
%     F = [A * xi == Z.b, p == Z.G * xi + Z.c];

end
for i = 1:length(Z.idx)
    F = [F, norm(xi(pointer + (1:Z.idx(i)),1), Z.type(i)) <= 1];
    pointer = pointer + Z.idx(i);
end

end