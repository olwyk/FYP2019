function H_str = hessianStructure(dmpc)

n = dmpc.n;
m = dmpc.m;
Np = dmpc.Np;

H_str = sparse( diag( ones(n*Np+m*Np,1), 0) + diag( ones(n*Np+m*Np-1,1), -1) + ...
    diag( ones(n*Np+m*Np-2,1), -2));

end