function idx = FindSet(Xj_list,x0)
% FindSet - Function that identifies the index of x0 in the sequence Xj_list 
%
% Syntax:  
%    idx = FindSet(Xj_list,x0)
%
% Inputs:
%    Xj_list - sequence of reachable sets 
%    x0 - initial state
%
% Outputs:
%    idx - index in the sequence where x0 belongs
%
% Other m-files required: compileCCG.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
ops = sdpsettings('solver','GUROBI');%, 'gurobi.BarConvTol', 1e-6);
vector = sdpvar(size(x0,1),size(x0,2)); 
in = {vector};

for i = 1 : size(Xj_list,1)
    % Compile the Yalmip constraint set and point
    [set,a] = compileCCG(Xj_list{i});
    out = {a};
    % Create the optimizer to search in each set
    find = optimizer([set, a == vector],0,ops,in,out);
    [~, flag] = find{x0};
%     disp(flag);
        if flag == 0
            idx = i;
            fprintf('x0 belongs to set idx = %i\n', idx)
            return
        end
end
idx = size(Xj_list,1);
disp('Does not belong to any set');
end

