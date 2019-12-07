function Dc_str = constraintsJacobianStructure(dmpc,Qi)
% Nonzero elements denote constraint derivatives that may be nozero at any
% point

Np = dmpc.Np;
No = size(dmpc.fixedObstacleCentres,2);
n = dmpc.n;
m = dmpc.m;

%%% Equality
Dc_str = sparse( zeros(n*Np+No*Np,n*Np+m*Np));
A = [zeros(3,3) ones(3,2) zeros(3,1); zeros(3,6)];
A(4,6) = 1;
Dc_str(1:n*Np,1:n*Np) = kron( eye(Np), eye(n)) + ...
                        kron( diag( ones(Np-1,1), -1), eye(n)+A);
Dc_str(1:n*Np,n*Np+1:end) = kron( eye(Np), [zeros(n-m,m); ones(m,m)]);
%%% Inequality
Dc_str(n*Np+1:end,1:n*Np) = repmat( kron( eye(Np), [ones(1,3) zeros(1,3)]), No, 1);

% Only keep activate constraints
Dc_str = Dc_str( [[1:n*Np]';n*Np+Qi],:);

end