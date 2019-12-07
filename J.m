function cost = J(dmpc,k,i,X,U,N,x_ref,Nv)
% dmpc: Struct containing problem-specific constants
% k : Current time instant
% i : Current agent
% X : All state vectors
% U : All control vectors
% N : Sequence of neighbouring agent vectors
% x_ref : Reference state vectors
% Nv : Sequence of neighbouring virtual agent vectors

Np = dmpc.Np;
Q =  dmpc.Q;
R =  dmpc.R;
S =  dmpc.S;
Sv = dmpc.Sv;
T =  dmpc.T;
V = dmpc.V;
Z = dmpc.Z;
epsln = dmpc.epsln;

cost = 0;
for p=0:Np-1
    X_kpi = X(1:3,k+p,i);
    Z_kpi = Z(1:3,k+p,i);
    U_kpi = U(:,k+p,i);
    % State trajectory tracking cost
    cost = cost + ( Z_kpi - X_kpi)'*Q*( Z_kpi - X_kpi);
    % Control input cost
    cost = cost + ( U_kpi)'*R*( U_kpi);
    % Set of neighbouring agents at the current time
    N_l = N{p+1};
    % Formation cost
    for idx=1:length(N_l)
        j = N_l( idx);
        cost = cost + 1./( ( X_kpi - X(1:3,k+p,j))'*S*( X_kpi - X(1:3,k+p,j)) + epsln);
    end
    % Set of neighbouring virtual agents at the current time
    Nv_l = Nv{p+1};
    if ~isempty( V)
        y_all = ( X_kpi - V(1:3, Nv_l));
    end
    % Dynamic obstacle cost
    for idx=1:length(Nv_l)
        y = y_all(:,idx);
        cost = cost + 1./( y'*Sv*y + epsln);
    end
end
% Terminal cost
cost = cost + ( Z(1:3,k+Np,i) - X(1:3,k+Np,i))'*T*( Z(1:3,k+Np,i) - X(1:3,k+Np,i));

end