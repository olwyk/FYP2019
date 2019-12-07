function DJi = gradientFn( dmpc,k,i,X,N,Nv,x_ref,x_opt,u_opt)

Np = dmpc.Np;
m = dmpc.m;
n = dmpc.n;
Q = dmpc.Q;
R = dmpc.R;
S = dmpc.S;
Sv = dmpc.Sv;
T = dmpc.T;
V = dmpc.V;
Z = dmpc.Z;
epsln = dmpc.epsln;

DJi = zeros(n*Np+m*Np,1);

% Non-terminal cost gradients
for p=0:Np-1
    %%% Control derivatives [0,Np-1]
    DuJi = 2*R*u_opt(:,p+1);
    % Fill the gradient vector
    DJi(n*Np+p*m+1:n*Np+p*m+ m) = DuJi;
    
    if p==0
        continue;
    end
    %%% State derivatives [1,Np-1]
    DxJi = -2*Q*( Z(1:3,k+p,i) - x_opt(1:3,p));
    % Set of neighbouring agents at the current time
    x_opt_p = x_opt(1:3,p);
    N_p = N{p+1};
    y_all = ( x_opt_p - X(1:3,k+p,N_p));
    for idx=1:length(N_p)
        y = y_all(:,idx);
        S_y = S*y;
        DxJi = DxJi - 2*S_y./( y'*S_y + epsln)^2;
    end
    % Set of virtual neighbouring agents at the current time
    N_p = Nv{p+1};
    if ~isempty( V)
        y_all = ( x_opt_p - V(1:3,N_p));
    end
    for idx=1:length(N_p)
        y = y_all(:,idx);
        Sv_y = Sv*y;
        DxJi = DxJi - 2*Sv_y/( y'*Sv_y + epsln)^2;
    end
    % Fill the gradient vector
    DJi([(p-1)*n+1:(p-1)*n+ 3]) = DxJi;
end

% Terminal cost gradient
DxJi = -2*T*( Z(1:3,k+Np,1) - x_opt(1:3,Np));
DJi([n*Np-5 n*Np-4 n*Np-3]) = DxJi;

end