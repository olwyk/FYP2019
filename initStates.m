function x0 = initStates(dmpc,k,i,X,U,u0)
% Initialise dynamical and control states.
% Note that the control sequence provided is assumed to be valid
% dmpc : Struct containing problem-specific constants
% k    : Iteration integer
% i    : Agent index
% X    : Matrix of all agent dynamical states
% U    : Matrix of all agent control inputs
% x0   : Set of initial dynamical states consistent with u0

Np = dmpc.Np;
dt = dmpc.dt;

U(:,1:Np,i) = u0;

for p=1:Np-1
    X(:,k+p,i) = X(:,k+p-1) + dt*f( dmpc, X(:,k+p-1,i), U(:,k+p-1,i));
end

% Return the initial set of resulting states
% This is shifted back by one since this is an initialisation only
x0 = X(:,k:k+Np-1,i);

end