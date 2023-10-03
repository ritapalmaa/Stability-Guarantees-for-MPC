function [F,p] = compileCCG(Z)
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

pointer = 0;
if isempty(Z.A)
    F = p == Z.G * xi + Z.c;
else
    F = [Z.A * xi == Z.b, p == Z.G * xi + Z.c];
end
for i = 1:length(Z.idx)
    F = [F, norm(xi(pointer + (1:Z.idx(i)),1), Z.type(i)) <= 1];
    pointer = pointer + Z.idx(i);
end

end