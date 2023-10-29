function list = BackwardReachabilitySets(RPI,X,U,model,nx,Nc,N)
% BackwardReachabilitySets - Function that calculates the backwards 
% sequence of reachable states 
%  
% Syntax:  
%    list = BackwardReachabilitySets(RPI,X,U,model,nx,Nc,N)
%
% Inputs:
%    RPI - Constrained Convex Generator for RPI
%    X - Constrained Convex Generator of state constraints
%    U - Constrained Convex Generator of input constraints
%    model - struct with the system model A and B matrices
%    nx - state dimensions
%    Nc - number of converge steps
%    N - prediction horizon
%
% Outputs:
%    list - sequence of reachable sets
%
% Other m-files required: CCGIntersect2Sets.m, CCGMinkowskiSum.m, CCGLinMap.m.
% Subfunctions: none
% MAT-files required: none

%------------- BEGIN CODE --------------

% Initialization
list = cell(Nc*N+1,1); 
Xj = RPI;

for i = 1 : (Nc*N)+1
    list{i} = Xj;
    Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A^-1,Xj,zeros(nx,1)),CCGLinMap(-(model.A^-1)*model.B,U,zeros(nx,1))); % est means estimate
    Xj = CCGIntersect2Sets(Xj_prev_est,X);
end
end