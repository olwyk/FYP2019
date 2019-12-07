function [posFeasBool] = isPositionInfeasible(dmpc,p)

posFeasBool = true;

No = dmpc.No;
Np = dmpc.Np;

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
    for k=1:Np
        c(k,i) = 1 - (p-xc)'*O*(p-xc);
    end
end

if any( any( c < 0))
    posFeasBool = false;
end

end