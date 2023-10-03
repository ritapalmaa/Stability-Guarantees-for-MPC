function set_reduced = SetReduction(maxError,X,nx,nu)
%SET_REDUCTION Summary of this function goes here
%   Detailed explanation goes here

E.G = maxError*eye(size(X.G,1),size(X.G,2)); E.c = zeros(nx,1);
E.A = zeros(0,nx); E.b = zeros(0,1); 
E.type = Inf; E.idx = size(X.G,2);

reduction_set = CCGLinMap(X.G',E,zeros(size(X.G,2),1));
red_box = boxCCG(reduction_set);
red_value = max(max(red_box.G));

% Calculate the maximum of the set
rad = cell(nx,1); idx = 1;
for i = 1 : nu : nx
    for j = 0 : nu-1
        rad{i + j} = norm(X.G(i:idx*nu,:),Inf);
    end
    idx = idx +1;
end

% Calculate the maximum of the set
rad = cell(nx,1); idx = 1;
for i = 1 : nu : nx
    for j = 0 : nu-1
        rad{i + j} = norm(X.G(i:idx*nu,:),Inf);
    end
    idx = idx +1;
end

% Calculate and compile the set reduced
set_reduced = CCGReduction(X,red_value,rad);
end

