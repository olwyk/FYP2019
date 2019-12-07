function [info] = checkConditions(dmpc, giveRecommendations)

Np = dmpc.Np;
Nc = dmpc.Nc;
dt = dmpc.dt;
A = dmpc.A;
B = dmpc.B;

maxAgentSpeed = max( -B(3,1)/A(3,3), -B(4,2)/A(4,4));
maxPtSpacing = maxAgentSpeed*dt;

% Find the maximum obstacle size
maxObsSize = 2*max(max( dmpc.fixedObstacleAxes));
minObsSize = 2*min(min( dmpc.fixedObstacleAxes));

% Ensure that the maximum obstacle size is less than the max avoidance
% radius
x0 = [0;0;maxAgentSpeed;maxAgentSpeed];
xNp = (eye(4)+dt*A)^(Np-1)*x0;
for k=1:Np
    xNp = xNp + (eye(4)+dt*A)^(k-1)*dt*B*[1;1];
end
avoidanceRadius = min( xNp(1:2));
if 2*avoidanceRadius < maxObsSize
    fprintf( 'Avoidance size is less than the maximum obstacle size.\n');
end
if giveRecommendations
    if 2*avoidanceRadius > 3*maxObsSize
        fprintf(['Avoidance radius large given obstacles. Consider:\n' ...
                 ' - Increasing obstacle size\n' ...
                 ' - Decreasing control horizon\n' ...
                 ' - Decreasing time step\n' ...
                 ' - Decreasing agent speed\n' ...
                 ' - Increasing prediction horizon\n']);
    end
end

% Ensure that the maximum point spacing is less than the minimum obstacle
% size
% Factor of 0.5 to account for smaller size edges of the ellipse
if maxPtSpacing > 0.25*minObsSize
    fprintf('Maximum point spacing too large. Decrease step size or max speed.\n');
end

info.maxAgentSpeed = maxAgentSpeed;
info.maxPtSpacing = maxPtSpacing;
info.minObsSize = minObsSize;
info.maxObsSize = maxObsSize;
info.avoidanceSize = 2*avoidanceRadius;

end