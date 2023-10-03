function Xj_list_obs = ForwardReachabilitySets(idx_obs,model,X,U,Xj_list,Obs)
%FORWARDREACHABILITYSETS Summary of this function goes here
%   Detailed explanation goes here

    nx = length(model.A);
    Xj_list_obs = cell(length(Xj_list),1); 
    
    for i = flip (1 : length(Xj_list)) 
        if i == length(Xj_list)-idx_obs
            Xj_list_obs{i} = CCGIntersect2Sets(Xj_list{i},Obs);
            Xj = Xj_list_obs{i};
        elseif i < length(Xj_list)-idx_obs
            Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A,Xj,zeros(nx,1)),CCGLinMap(model.B,U,zeros(nx,1))); % est means estimate
            Xj = CCGIntersect2Sets(Xj_prev_est,Xj_list{i}); 
            Xj_list_obs{i} = Xj;
        else
            Xj_list_obs{i} = Xj_list{i};
        end
    end

end

