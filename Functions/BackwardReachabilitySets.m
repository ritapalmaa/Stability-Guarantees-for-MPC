function list = BackwardReachabilitySets(RPI,X,U,model,nx,N_c,steps,N)
% BackwardReachabilitySets - Function that calculates the backwards 
% sequence of reachable states 
%  
% Syntax:  
%    list = BackwardReachabilitySets(RPI,X,U,model,nx,N_c,steps,N)
%
% Inputs:
%    RPI - Constrained Convex Generator for RPI
%    X - Constrained Convex Generator of state constraints
%    U - Constrained Convex Generator of input constraints
%    model - struct with the system model A and B matrices
%    nx - state dimensions
%    N_c - number of converge steps
%    steps - number of simulation steps
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
list = cell(steps+1,1); 
Xj = RPI;

for i = 1 : (N_c*N) +1
    list{i} = Xj;
    Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A^-1,Xj,zeros(nx,1)),CCGLinMap(-(model.A^-1)*model.B,U,zeros(nx,1))); % est means estimate
    Xj = CCGIntersect2Sets(Xj_prev_est,X);
end
end

