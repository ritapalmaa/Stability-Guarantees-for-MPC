function list = BackwardReachabilitySets(RPI,X,U,model,nx,convergence_steps,simulation_steps,N)
%BACKWARDREACHABILITYSETS Summary of this function goes here
%   Detailed explanation goes here
    list = cell(simulation_steps+1,1); 
    Xj = RPI;

    for i = 1 : (convergence_steps*N) +1
        list{i} = Xj;
        Xj_prev_est = CCGMinkowskiSum(CCGLinMap(model.A^-1,Xj,zeros(nx,1)),CCGLinMap(-(model.A^-1)*model.B,U,zeros(nx,1))); % est means estimate
        Xj = CCGIntersect2Sets(Xj_prev_est,X);
    end

end

