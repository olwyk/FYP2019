function H = hessian(dmpc,k,x,N,Nv,X,sigma,lambda,Qi)

n = dmpc.n;
m = dmpc.m;
Np = dmpc.Np;
Q = dmpc.Q;
R = dmpc.R;
S = dmpc.S;
Sv = dmpc.Sv;
T = dmpc.T;
epsln = dmpc.epsln;
VT = dmpc.VT;
g = dmpc.g;
No = dmpc.No;
V = dmpc.V;

% persistent Gzeros;
% Gzeros = sparse(n*Np+m*Np, n*Np+m*Np);
x_opt = reshape( x(1:n*Np), n, Np);

%%% Objective Hessian
HJxx = zeros(n*Np);
for p=1:Np-1
    x_opt_p = x_opt(1:3,p);
    alpha = 2*Q;
    % Set of neighbouring agents at the current time
    N_p = N{p+1};
    y_all = ( x_opt_p - X(1:3,k+p, N_p));
    for idx=1:length(N_p)
        y = y_all(:,idx);
        S_y = S*y;
        b = y'*S*y + epsln;
        alpha = alpha + ( 8*(S_y)*(S_y)' - 2*S*b)/b^3;
    end
    % Set of neighbouring virtual agents
    N_p = Nv{p+1};
    if ~isempty( V)
        y_all = ( x_opt_p - V(:,N_p));
    end
    for idx=1:length(N_p)
        y = y_all(:,idx);
        Sv_y = Sv*y;
        b = y'*Sv*y + epsln;
        alpha = alpha + ( 8*(Sv_y)*(Sv_y)' - 2*Sv*b)/b^3;
    end
    
    HJxx( (p-1)*n+1:(p-1)*n+3, (p-1)*n+1:(p-1)*n+3) = alpha;
end
HJxx( (Np-1)*n+1:(Np-1)*n+3, (Np-1)*n+1:(Np-1)*n+3) = 2*T;
HJuu = kron( eye(Np),2*R);
H = [HJxx             zeros(n*Np,m*Np);
     zeros(m*Np,n*Np) HJuu];
H = sparse( sigma*tril( H));

 
%%% Constraint Hessians
%%%% Equality constraints
i = n+1;
for p=1:Np-1
    psi =   x_opt(4,p);
    gamma = x_opt(5,p);
    phi =   x_opt(6,p);
    
    G1 = spalloc(n*Np+m*Np, n*Np+m*Np, 3);
    G1( (p-1)*n+4, (p-1)*n+4) = +VT*cos(gamma)*cos(psi);
    G1( (p-1)*n+5, (p-1)*n+4) = -VT*sin(gamma)*sin(psi);
    G1( (p-1)*n+5, (p-1)*n+5) = +VT*cos(gamma)*cos(psi);
    H = H + lambda(i)*G1;
    i = i + 1;
    
    G2 = spalloc(n*Np+m*Np, n*Np+m*Np, 3);
    G2( (p-1)*n+4, (p-1)*n+4) = +VT*cos(gamma)*sin(psi);
    G2( (p-1)*n+5, (p-1)*n+4) = +VT*sin(gamma)*cos(psi);
    G2( (p-1)*n+5, (p-1)*n+5) = +VT*cos(gamma)*sin(psi);
    H = H + lambda(i)*G2;
    i = i + 1;
    
    G3 = spalloc(n*Np+m*Np, n*Np+m*Np, 1);
    G3( (p-1)*n+5, (p-1)*n+5) = -VT*sin(gamma);
    H = H + lambda(i)*G3;
    i = i + 1;
    
    G4 = spalloc(n*Np+m*Np, n*Np+m*Np, 1);
    G4( (p-1)*n+6, (p-1)*n+6) = -2*g/VT*tan(phi)*sec(phi)^2;
    H = H + lambda(i)*G4;
    i = i + 1;
    
    % The last two are direct control inputs and hence have Hessians of
    % only zeros
    i = i + 2;
end
%%%% Inequality constraints
for j=1:No
    O = [-2/dmpc.fixedObstacleAxes(1,j)^2 0 0;
         0 -2/dmpc.fixedObstacleAxes(2,j)^2 0;
         0 0 -2/dmpc.fixedObstacleAxes(3,j)^2];
    for p=1:Np
        % Continue using the index i from the previous loop
        if any( (j-1)*Np+p == Qi)
            G = zeros(n*Np+m*Np);
            G( (p-1)*n+1:(p-1)*n+3, (p-1)*n+1:(p-1)*n+3) = tril( O);
            H = H + lambda(i)*G;
            i = i + 1;
        end
    end
end

H = sparse( H);
end