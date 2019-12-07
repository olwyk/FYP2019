function [] = animateF16Tracking(figH,dmpc,X,xhat,k)

figure( figH); clf; hold on;

Np = dmpc.Np;
k_max = dmpc.k_max;
i = 1;
X1 = reshape( X(1,:,:), k_max, []);
X2 = reshape( X(2,:,:), k_max, []);
X3 = reshape( X(3,:,:), k_max, []); 
xhat1 = reshape( xhat(7,:,:), k_max, []);
xhat2 = reshape( xhat(8,:,:), k_max, []);
xhat3 = reshape( xhat(9,:,:), k_max, []); 



% Ribbon
phi = -( xhat(12,2:k,1)-xhat(12,1:k-1,1));
phi(1) = phi(1) + pi/2;
ribbonH = streamribbon( {[xhat1(1:k-1,1), xhat2(1:k-1,1), xhat3(1:k-1,1)]},  {phi'}, 5);
ribbonH.FaceAlpha = 1.0;
shading interp;

% Aircraft
aircraftModel = stlread( 'f-16.stl');
h(i) = trisurf( aircraftModel);
transformObj{i} = hgtransform('Parent', gca);
set(h(i),'Parent',transformObj{i});
Rx = makehgtform('xrotate',-xhat(12,k,i));
Ry = makehgtform('yrotate',-xhat(11,k,i));
Rz = makehgtform('zrotate',wrapToPi( +xhat(10,k,i)));
Rmodel = makehgtform('zrotate',pi);
T = makehgtform('translate',xhat(7:9,k,i));
S = makehgtform('scale',[1,1,1]);
set(transformObj{i},'Matrix',eye(4));
set(transformObj{i},'Matrix',T*S*Rz*Ry*Rx*Rmodel); 

% Prediction
plot3( X1(k+1:k+Np,i), X2(k+1:k+Np,i), X3(k+1:k+Np), 'r-s');

dX = 700;
% axis([xhat1(k,i)-dX xhat1(k,i)+dX xhat2(k,i)-dX xhat2(k,i)+dX xhat3(k,i)-dX xhat3(k,i)+dX]);
axis equal;
axis([xhat1(k,i)-dX xhat1(k,i)+dX xhat2(k,i)-dX xhat2(k,i)+dX]);


end