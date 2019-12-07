function [c,ceq] = optConstraints(dmpc, x_opt, u_opt, xk, Qi)
% dmpc  : Struct containing problem-specific constants
% x_opt : Optimal sequence of dynamical states
% u_opt : Optimal sequence of control inputs
% xk    : The kth state; used to enforce x(k|k) = x(k)

Np = dmpc.Np;
Nc = dmpc.Nc;
No = dmpc.No;
dt = dmpc.dt;
n = dmpc.n;
m = dmpc.m;

% Add the initial (kth) state to the start of the state sequence
x_opt_pad = [xk x_opt];

u_opt_pad = u_opt;


%%% Inequality Constraints
r = dmpc.fixedObstacleCentres;
s = dmpc.fixedObstacleAxes;
c = zeros(Np,No);
for i=1:No
    % x: Position
    % r: Ellipsoid centre
    % s: Ellipsoid axes
    a1 = s(1,i);
    a2 = s(2,i);
    a3 = s(3,i);
    O = [1/a1^2 0      0; 
         0      1/a2^2 0;
         0      0      1/a3^2];
    xc = r(:,i);
    for p=1:Np
        x = x_opt(1:3,p);
        c(p,i) = 1 - (x-xc)'*O*(x-xc);
    end
end

%%% Equality Constraints
% Ensure that the agent moves according to its dynamics
% Note that x* = {xk, xk+1, ... xk+Np} due to padding
ceq = zeros(n,Np);
for p=1:Np     
    ceq(:,p) = ( x_opt_pad(:,p+1) - x_opt_pad(:,p))/dt - ... % finite difference approx.   
               ( f( dmpc, x_opt_pad(:,p), u_opt_pad(:,p)));  % agent dynamics
end

% Reshaped as a column vector for IPOPT, with equality constraints first
c = [reshape(ceq,[],1); reshape(c,[],1)];

% Only keep active inequality constraints
c = [c(1:n*Np); c(n*Np+Qi)];

end