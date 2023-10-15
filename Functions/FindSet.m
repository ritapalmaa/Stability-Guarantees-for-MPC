function idx = FindSet(list,x)
% FindSet - Function that identifies the index of x in the sequence Xj_list 
%
% Syntax:  
%    idx = FindSet(Xj_list,x)
%
% Inputs:
%    list - sequence of reachable sets 
%    x - state
%
% Outputs:
%    idx - index in the sequence where x0 belongs
%
% Other m-files required: compileCCG.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
ops = sdpsettings('solver','gurobi');
vector = sdpvar(size(x,1),size(x,2)); 
in = {vector};

for i = 1 : size(list,1)
    % Compile the Yalmip constraint set and point
    [set,a] = compileCCG(list{i});
    out = {a};
    % Create the optimizer to search in each set
    find = optimizer([set, a == vector],0,ops,in,out);
    [~, flag] = find{x};
        if flag == 0
            idx = i;
            fprintf('x0 belongs to set idx = %i\n', idx)
            return
        end
end
idx = 0;
disp('Does not belong to any set');
end

