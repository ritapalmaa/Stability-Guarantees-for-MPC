function Z = CCGReduction(Z,value,d)
%CCGREDUCTION Summary of this function goes here
%   Detailed explanation goes here

for i = 1 : size(Z.G,1)
    for j = 1:size(Z.G,2)
            Z.G(i,j) = Z.G(i,j) * (1-value/d{i});
    end
end

end

