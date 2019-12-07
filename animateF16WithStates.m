function [] = animateF16WithStates( figH, dmpc, i, xhat)

dt = dmpc.dt;
aircraftModel = stlread( 'f-16.stl');
figure( figH);

for k=1:size( xhat, 2)
    hold off; clf;
    
    tiledlayout(2,3);
    
    % VT,Pe plot
    nexttile;
    hold on;
    plot( 0:dt:k*dt, vecnorm( xhat(1:3,1:k,i), 1));
    plot( 0:dt:k*dt, xhat(13,1:k,i));
    legend('Airspeed, V_T [m/s]','Engine Power, P_e [%]');
    xlabel('Time, t [s]');
    
    % Aircraft marker
    nextttile([])
    transformObj{1} = hgtransform('Parent', gca);
    h(1) = trisurf( aircraftModel);
    set(h(1),'Parent',transformObj{1});
    Rx = makehgtform('xrotate',-xhat(10,k,i));
    Ry = makehgtform('yrotate',-xhat(11,k,i));
    Rz = makehgtform('zrotate',wrapToPi( +xhat(12,k,i)));
    Rmodel = makehgtform('zrotate',pi);
    T = makehgtform('translate',xhat(7:9,k,i));
    S = makehgtform('scale',[1,1,1]*0.3048*50);
    set(transformObj{i},'Matrix',eye(4));
    set(transformObj{i},'Matrix',T*S*Rz*Ry*Rx*Rmodel);

    % 
end

end