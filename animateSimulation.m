function frame = animateSimulation(figH, dmpc, X, ref, xhat, animOptions)

viewLimits = animOptions.viewLimits;
trackView = animOptions.trackView;
is2D = animOptions.is2D;
freezeFrame = animOptions.freezeFrame;
showK = animOptions.showK;
showPts = animOptions.showPts;
useF16Kinematics = animOptions.useF16Kinematics;
ptSize = animOptions.ptSize;

figure( figH); clf;

k_max = dmpc.k_max;
dt = dmpc.dt;
Na = dmpc.Na;
No = dmpc.No;
X1 = reshape( X(1,:,:), k_max, []);
X2 = reshape( X(2,:,:), k_max, []);
X3 = reshape( X(3,:,:), k_max, []); 
if dmpc.useF16Kinematics
    xhat1 = reshape( xhat(7,:,:), k_max, []);
    xhat2 = reshape( xhat(8,:,:), k_max, []);
    xhat3 = reshape( xhat(9,:,:), k_max, []); 
end
V = dmpc.V;
vertices = dmpc.detVertices;
Ndo = dmpc.Ndo;

% Aircraft object
aircraftModel = stlread( 'f-16.stl');

if freezeFrame
    k_init = showK;
    k_max = showK;
else
    k_init = 1;
end

for k=k_init:k_max
    hold off;
    clf;
    if is2D
        if trackView
            X1Min = min( X1(k,:));
            X1Max = max( X1(k,:));
            X1Avg = ( X1Min + X1Max)/2;
            dX1 = X1Max - X1Min;
            X2Min = min( X2(k,:));
            X2Max = max( X2(k,:));
            X2Avg = ( X2Min + X2Max)/2;
            dX2 = X2Max - X2Min;
            dX = 3*max( dX1, dX2);
            if Na==1 || dX==0
                dX = 500;
            end
            viewLimits = [X1Avg-dX/2 X1Avg+dX/2 ...
                          X2Avg-dX/2 X2Avg+dX/2];
        end
        axis(viewLimits(1:4));
    else
        if trackView
            X1Min = min( X1(k,:));
            X1Max = max( X1(k,:));
            X1Avg = ( X1Min + X1Max)/2;
            dX1 = X1Max - X1Min;
            X2Min = min( X2(k,:));
            X2Max = max( X2(k,:));
            X2Avg = ( X2Min + X2Max)/2;
            dX2 = X2Max - X2Min;
            X3Min = min( X3(k,:));
            X3Max = max( X3(k,:));
            X3Avg = ( X3Min + X3Max)/2;
            dX3 = X3Max - X3Min;
            dX = 10*max( [dX1 dX2 dX3]);
            if Na==1 || dX == 0
                dX = 500;
            end
            viewLimits = [X1Avg-dX/2 X1Avg+dX/2 ...
                          X2Avg-dX/2 X2Avg+dX/2 ...
                          max(0,X3Avg-dX/2) X3Avg+dX/2];
            axis(viewLimits);
        else
            X3Max = max( X3(k,:));
            zlim([0 X3Max]);
            view(3);
        end
        
    end
    
    
    hold on;
    grid on;
    % Curve for all time
    if showPts
%         plot3( X1( 1:k,:), X2( 1:k,:), X3( 1:k,:),'-o', ...
%             'Color',[177, 205, 249]/255, 'MarkerSize', 5, ...
%             'MarkerEdgeColor', 'w', 'MarkerFaceColor', );
        for i=1:Na
            leg(1) = scatter3( X1( 1:k,i), X2( 1:k,i), X3( 1:k,i), ...
                ptSize, [0,0,0]/255, '^', 'filled');
        end
    end
%     if showPts
%         plot3( X1( 1:k,:), X2( 1:k,:), X3( 1:k,:),'-o', ...
%             'Color',[177, 205, 249]/255, 'MarkerSize', 5, ...
%             'MarkerEdgeColor', 'w', 'MarkerFaceColor', [155, 191, 250]/255);
%     else
%         plot3( X1( 1:k,:), X2( 1:k,:), X3( 1:k,:),'-', ...
%             'Color',[177, 205, 249]/255);
%     end
%     plot3( X1( 1:k,:), X2( 1:k,:), X3( 1:k,:),'k-');
%     % Curve to this point, using the last 20 points
%     plot3( X1( max(1,k-20):k,:), X2( max(1,k-20):k,:), X3( max(1,k-20):k,:),...
%         'Color',[0, 0.4470, 0.7410]);
    % Aircraft ribbon
%     for i=1:Na
%         if k==1
%             break;
%         end
%         phi = -( X(6,2:k,i)-X(6,1:k-1,i));
%         phi(1) = phi(1) + pi/2;
%         ribbonH = streamribbon( {[X1(1:k-1,i), X2(1:k-1,i), X3(1:k-1,i)]},  {phi'}, 5);
%         ribbonH.FaceAlpha = 1.0;
%         shading interp;
%     end
    % Interaction lines
    if showPts
        for i=1:Na
            for j=i+1:Na
                if dmpc.history.Adj(i,j,k) > 0
                    leg(2) = plot3( [X1(k,i) X1(k,j)], [X2(k,i) X2(k,j)], [X3(k,i) X3(k,j)], ...
                        '--', 'Color', '#f7bc25');
                end
            end
        end
    end
    % Reference
    for i=1:size(ref,2)
        leg(3) = scatter3( ref(1,i), ref(2,i), ref(3,i), 200, [0.4660 0.6740 0.1880],...
            'd','filled','MarkerFaceColor', [0.4660 0.6740 0.1880], ...
            'MarkerEdgeColor', 'w');
        plot3( ref(1,i)*[1 1], ref(2,i)*[1 1], ref(3,i)*[0 1], ...
            '-.', 'Color', [0.4660 0.6740 0.1880]);
    end
    % Hard obstacles
    r = dmpc.fixedObstacleCentres;
    s = dmpc.fixedObstacleAxes;
    for i=1:No
        [xo,yo,zo] = ellipsoid( r(1,i),r(2,i),r(3,i),s(1,i),s(2,i),s(3,i));
        leg(4) = surf(xo,yo,-zo,xo.*0,'FaceAlpha',1.,'EdgeColor','none');
    end
    % Soft obstacles
    if Ndo > 0
        leg(5) = patch( vertices.x, vertices.y, -vertices.z, 'c');
        Nv = dmpc.history.detectedVirtualAgents{k};
        leg(8) = scatter3( V(1,Nv), V(2,Nv), -V(3,Nv), 'r.');
    end
    % Actual aircraft path
    if useF16Kinematics && showPts
        for i=1:Na
            leg(6) = scatter3( xhat1(1:k,i), xhat2(1:k,i), xhat3(1:k,i), ...
                ptSize, 'r', 'o', 'filled');
        end
    end
    % Predicted path
    if showPts
        for i=1:Na
            P1 = reshape( dmpc.P(k,1,:,i), 1, []);
            P2 = reshape( dmpc.P(k,2,:,i), 1, []);
            P3 = -reshape( dmpc.P(k,3,:,i), 1, []);
            leg(7) = scatter3( P1, P2, P3, 1.5*ptSize, [195, 59, 245]/255, 's', 'filled');
        end
    end
    % Aircraft marker
    if useF16Kinematics
        for i=1:Na
            transformObj{i} = hgtransform('Parent', gca);
            h(i) = trisurf( aircraftModel);
            set(h(i),'Parent',transformObj{i});
    %         Rx = makehgtform('xrotate',-X(6,k,i));
    %         Ry = makehgtform('yrotate',-X(5,k,i));
    %         Rz = makehgtform('zrotate',wrapToPi( +X(4,k,i)));
    %         Rmodel = makehgtform('zrotate',pi);
    %         T = makehgtform('translate',X(1:3,k,i));
    %         S = makehgtform('scale',[1,1,1]*0.3048*100);
    %         set(transformObj{i},'Matrix',eye(4));
    %         set(transformObj{i},'Matrix',T*S*Rz*Ry*Rx*Rmodel);
            Rx = makehgtform('xrotate',-xhat(10,k,i));
            Ry = makehgtform('yrotate',-xhat(11,k,i));
            Rz = makehgtform('zrotate',wrapToPi( +xhat(12,k,i)));
            Rmodel = makehgtform('zrotate',pi);
            T = makehgtform('translate',xhat(7:9,k,i));
            S = makehgtform('scale',[1,1,1]*0.3048*50);
            set(transformObj{i},'Matrix',eye(4));
            set(transformObj{i},'Matrix',T*S*Rz*Ry*Rx*Rmodel);
            plot3( xhat1(k,i)*[1 1], xhat2(k,i)*[1 1], xhat3(k,i)*[0 1], 'k-.');
        end
    end
    % Height indicator
%     if is2D
%         for i=1:Na
%             text( X1(k,i)+2, X2(k,i), sprintf('%3.0f m', X3(k,i)));
%         end
%     end
    % Title block
    title( sprintf('k=%i; t=%.1f s', k, k*dt), 'FontSize', 18);
    if ~trackView
        axis equal;
        zlim([0 inf]);
    end
    xlabel('x [m]', 'FontSize', 18);
    ylabel('y [m]', 'FontSize', 18);
    zlabel('h [m]', 'FontSize', 18);
    figH = figure( figH);
    
%     legend( leg([1 3 6 7 5]), 'MPC Trajectory', 'Target Point', 'Aircraft Position', 'Predicted Trajectory', 'Soft Obstacle',  ...
%         'Location', 'NorthWest', 'FontSize', 18);
%     legend( leg([1 3 6 7 4]), 'MPC Trajectory', 'Target Point', 'Aircraft Position', 'Predicted Trajectory', 'Hard Obstacle', ...
%         'Location', 'NorthWest');
%     legend( leg([3 5]), 'Target Point', 'Soft Obstacle', ...
%         'Location', 'NorthWest', 'FontSize', 18);
%     if showFinal && showPts
%         legend(leg, 'Past planned trajectory', 'Inter-aircraft communication', 'Target point', ...
%             'Hard obstacle', 'Soft obstacle', 'F-16 trajectory', 'Predicted trajectory');
%     end
    drawnow
    frame(k) = getframe( figH);
end

end