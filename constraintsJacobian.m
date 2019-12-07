function Dc = constraintsJacobian(dmpc,x_opt,Qi)
% Required when constraints are given to IPOPT
% Output format:
% - MxN sparse matrix, where M is the no. constraints and N is the no.
%   variables

dt = dmpc.dt;
n = dmpc.n;
Np = dmpc.Np;
No = dmpc.No;
m = dmpc.m;
fixedObstacleAxes = dmpc.fixedObstacleAxes;
fixedObstacleCentres = dmpc.fixedObstacleCentres;

x_opt = reshape( x_opt(1:n*Np), n, Np);

% Equality constraints come first (easier this way since they are fixed
% size)
% Sparse matrix to match IPOPT requirements
Dc = sparse( zeros(n*Np+No*Np,n*Np+m*Np));
%%% Equality constraints
for p=1:Np-1
    psi =   x_opt(4,p);
    gamma = x_opt(5,p);
    phi =   x_opt(6,p);
    VT =    dmpc.VT;
    g =     dmpc.g;
    Dc( p*n+1:p*n+4, (p-1)*n+4:(p-1)*n+n) = -...
        [-VT*sin(psi)*cos(gamma) -VT*cos(psi)*sin(gamma) +0; ...
         +VT*cos(psi)*cos(gamma) -VT*sin(psi)*sin(gamma) +0; ...
         +0                      -VT*cos(gamma)          +0; ...
         +0                      +0                      +g/VT*sec(phi)^2];
end
Dc(1:n*Np,1:n*Np) = Dc(1:n*Np,1:n*Np) + kron( eye(Np), eye(n)/dt) + ...
                        kron( diag( ones(Np-1,1), -1), -eye(n)/dt);
Dc(1:n*Np,n*Np+1:n*Np+m*Np) = kron( eye(Np), [zeros(n-m,m); -eye(m,m)]);
%%% Inequality constraints
for i=1:No
    a = fixedObstacleAxes(1,i);
    b = fixedObstacleAxes(2,i);
    c = fixedObstacleAxes(3,i);
    O = [1/a^2 0     0; 
         0     1/b^2 0;
         0     0     1/c^2];
    xc = fixedObstacleCentres(:,i);
    allRows = -2*( x_opt(1:3,:) - xc)'*O;
    for p=1:Np
        Dc(n*Np+(i-1)*Np+p, 1+(p-1)*n:p*n) = [allRows(p,:) zeros(1,3)];
    end
end

% Only keep activate constraints
Dc = Dc([ [1:n*Np]';n*Np+Qi],:);

end