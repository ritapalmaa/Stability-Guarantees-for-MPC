function [model,X,U,D,RPI,lqr,Acl] = Init2D(nx,nu,Ts)
%INIT2D Summary of this function goes here
%   Detailed explanation goes here
    
    % Discret Plant Model
    model.A = [eye(nu) eye(nu)*Ts eye(nu)*(Ts^2/2);    
           zeros(nu) eye(nu) eye(nu)*Ts;
           zeros(nu) zeros(nu) eye(nu)];
    model.B = [eye(nu)*Ts^3/6 eye(nu)*Ts^2/2  eye(nu)*Ts]';
    
    % Sets
    X.G = 100*eye(nx); X.c = zeros(nx,1);
    X.A = zeros(0,nx); X.b = zeros(0,1); 
    X.type = Inf; X.idx = nx;

    U.G = 5*eye(nu); U.c = zeros(nu,1);
    U.A = zeros(0,nu); U.b = zeros(0,1); 
    U.type = 2; U.idx = nu;

    D.G = [zeros(nu*nu,nu); 0.5*eye(nu)]; D.c = zeros(nx,1);
    D.A = zeros(0,nx); D.b = zeros(0,1);
    D.type = Inf; D.idx = nu; 

    % LQR Model
    lqr.Q = diag(2*ones(1,nx)); lqr.R = eye(nu);
    [lqr.K,lqr.P,~] = dlqr(model.A,model.B,lqr.Q,lqr.R);
    Acl = model.A - model.B * lqr.K;

    % RPI = CCGInnerRPI(Acl, D, N); 
    % using simple RPI made by hand
    RPI.G = 0.1*eye(nx); RPI.c = zeros(nx,1); 
    RPI.A = zeros(0,nx); RPI.b = zeros(0,1); 
    RPI.type = Inf; RPI.idx = nx;
end

