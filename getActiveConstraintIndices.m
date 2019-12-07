function [Qi,collisionFound] = getActiveConstraintIndices(dmpc,x_opt)
% Finds the set of indices for active inequality constraints

No = dmpc.No;
Np = dmpc.Np;
fixedObstacleAxes = dmpc.fixedObstacleAxes;
fixedObstacleCentres = dmpc.fixedObstacleCentres;
constraintEpsln = dmpc.constraintEpsln;

collisionFound = false;

Qi = [];
for j=1:No
    rx = fixedObstacleAxes(1,j);
    ry = fixedObstacleAxes(2,j);
    rz = fixedObstacleAxes(3,j);
    O = [1/rx^2 0      0;
         0      1/ry^2 0;
         0      0      1/rz^2];
    xc = fixedObstacleCentres(1:3,j);
    constraintFn = @(x) 1 - (x-xc)'*O*(x-xc);
    for p=1:Np
        if abs( constraintFn( x_opt(1:3,p))) < constraintEpsln
            Qi = [Qi; (j-1)*Np+p];
        end
        if constraintFn( x_opt(1:3,p)) > 1e-3  % solver tolerance 1e-4
            collisionFound = true;
%             constraintFn( x_opt(1:3,p))
        end
    end
end

end