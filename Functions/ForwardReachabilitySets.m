function Xj_list_obs = ForwardReachabilitySets(idx,model,U,Xj_list,O)
% ForwardReachabilitySets - Function that calculates the sequence of 
% reachable states adjusting these sets when an obstacle is faced
%  
% Syntax:  
%    Xj_list_obs = ForwardReachabilitySets(idx_obs,model,X,U,Xj_list,Obs)
%
% Inputs:
%    idx - index in the sequence where obstacle is faced
%    model - struct with the system model A and B matrices
%    X - Constrained Convex Generator of state constraints
%    U - Constrained Convex Generator of input constraints
%    Xj_list - sequence of reachable sets 
%    O - Constrained Convex Generator for the obstacle
%
% Outputs:
%    Xj_list_obs - sequence of reachable sets with obstcale introduced
%
% Other m-files required: CCGIntersect2Sets.m, CCGMinkowskiSum.m, CCGLinMap.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
nx = length(model.A);
Xj_list_obs = cell(length(Xj_list),1); 

% Iteration through Xj_list in reverse order
for i = flip (1 : length(Xj_list)) 
    % Case 1. Handling the Obstacle
    if i == length(Xj_list)-idx
        Xj_list_obs{i} = CCGIntersect2Sets(Xj_list{i},O);
        Xj = Xj_list_obs{i};
    % Case 2. Adjusting Reachability Sets after Obstacle
    elseif i < length(Xj_list)-idx
        Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A,Xj,zeros(nx,1)),CCGLinMap(model.B,U,zeros(nx,1))); % est means estimate
        Xj = CCGIntersect2Sets(Xj_prev_est,Xj_list{i}); 
        Xj_list_obs{i} = Xj;
    % Case 3. No Adjustments before Obstacle
    else
        Xj_list_obs{i} = Xj_list{i};
    end
end
end

