function idx = FindSet(Xj_list,x0)
%FIND_SET Find the set for which x0 belong
    ops = sdpsettings('solver','GUROBI');
    vector = sdpvar(size(x0,1),size(x0,2)); 
    in = {vector};
    
    for i = 1 : size(Xj_list,1)
        [set,a] = compileCCG(Xj_list{i});
        out = {a};
        confirmation = optimizer([set, a == vector],0,ops,in,out);
        [~, flag] = confirmation{x0};
            if flag == 0
                idx = i;
                fprintf('x0 belongs to set idx = %i\n', idx)
%                 disp('x0 belongs to this set');
                return
            end
    end
end

