function [V,vertices] = generateVirtualAgentStates( dmpc, detectedObstacles, w)
% dynamicObstacleCorners : Matrix of all obstacle corners defining the
%                          polygons
% w                      : Width between adjacent virtual agents (min.)

Ndo = length( detectedObstacles); % Num dynamic obstacles
No = dmpc.No;

%%% Soft obstacles
V = [];
vertices.x = zeros(4*6,Ndo);
vertices.y = zeros(4*6,Ndo);
vertices.z = zeros(4*6,Ndo);
for i=1:Ndo
    thisObstacle = detectedObstacles{i};
    dx = thisObstacle(1,2);
    dy = thisObstacle(2,2);
    dz = thisObstacle(3,2);
    
    % Create a filled volume of all the points, then remove those that
    % aren't along the boundaries
    [x,y,z] = meshgrid(-dx:w:dx, -dy:w:dy, -dz:w:dz);
    x = reshape(x,[],1);
    y = reshape(y,[],1);
    z = reshape(z,[],1);
    idx_x = ( x==-dx | x==dx);
    idx_y = ( y==-dy | y==dy);
    idx_z = ( z==-dz | z==dz);
    x = x( idx_x | idx_y | idx_z);
    y = y( idx_x | idx_y | idx_z);
    z = z( idx_x | idx_y | idx_z);
    
    xc = thisObstacle(1,1);
    yc = thisObstacle(2,1);
    zc = thisObstacle(3,1);
    
    V = [V [x'+xc;y'+yc;z'+zc]];
    
    vertices.x(:,i) = xc+[-dx;dx;dx;-dx;...  % bottom 
                          -dx;dx;dx;-dx;...  % top
                          -dx;-dx;dx;dx;...  % left
                          -dx;-dx;dx;dx;...  % right
                          -dx;-dx;-dx;-dx;...% back
                          dx;dx;dx;dx];      % front
    vertices.y(:,i) = yc+[-dy;-dy;dy;dy;...
                          -dy;-dy;dy;dy;...
                          -dy;-dy;-dy;-dy;...
                          dy;dy;dy;dy;...
                          -dy;-dy;dy;dy;...
                          -dy;-dy;dy;dy];
    vertices.z(:,i) = zc+[-dz;-dz;-dz;-dz;...
                          dz;dz;dz;dz;...
                          -dz;dz;dz;-dz;...
                          -dz;dz;dz;-dz;...
                          -dz;dz;dz;-dz;...
                          -dz;dz;dz;-dz];
end
vertices.x = reshape(vertices.x, 4,[]);
vertices.y = reshape(vertices.y, 4,[]);
vertices.z = reshape(vertices.z, 4,[]);


%%% Hard obstacle centre repulsion
% for i=1:No
%     r = dmpc.fixedObstacleAxes(:,i);
%     c = dmpc.fixedObstacleCentres(:,i);
%     V = [V c c+[1 0 0]*r c+[-1 0 0]*r c+[0 1 0]*r c+[0 -1 0]*r ...
%         c+[0 0 1]*r c+[0 0 -1]*r];
% end

end