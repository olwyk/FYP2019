%% Only run this once!
% x_traj = zeros(6,k);
% x_traj(1,:) = X(1,1:k);
% x_traj(2,:) = X(2,1:k);
% x_traj(3,:) = X(3,1:k);
% x_traj(4,:) = X(6,1:k);
% x_traj(5,:) = X(5,1:k);
% x_traj(6,:) = X(4,1:k);


%%
dt = 0.05;
VT = 250;

show3d = true;

y0 = [250;0;0; 0;0;0; 0;0;2500; 0*pi/180;0*pi/180;0*pi/180; 20];
f16_odefn_wrapper = @(t,y) f16_odefn(1+floor(t/dt), y, x_traj, VT);
[t,y] = ode45(f16_odefn_wrapper, [0:dt:9.9], y0);
y = y';
U = zeros(4, length(t));
for i=1:length(t)
    [~,U(:,i)] = f16_odefn_wrapper(t(i),y(:,i)');
end

u = y(1,:);
v = y(2,:);
w = y(3,:);
p = y(4,:);
q = y(5,:);
r = y(6,:);
X = y(7,:);
Y = y(8,:);
h = y(9,:);
phi = y(10,:);
theta = y(11,:);
psi = y(12,:);
P = y(13,:);

VT = vecnorm([u;v;w]);

figure(1); clf;
subplot(6,3,1);
plot( t, u);
title('U');
grid on;
subplot(6,3,2);
plot( t, v);
title('V');
grid on;
subplot(6,3,3);
plot( t, w);
title('W');
grid on;

subplot(6,3,4);
plot( t, phi*180/pi);
title('\phi');
grid on;
subplot(6,3,5);
plot( t, theta*180/pi);
title('\theta');
grid on;
subplot(6,3,6);
plot( t, psi*180/pi);
title('\psi');
grid on;

subplot(6,3,7);
plot( t, p);
title('P');
grid on;
subplot(6,3,8);
plot( t, q);
title('Q');
grid on;
subplot(6,3,9);
plot( t, r);
title('R');
grid on;

subplot(6,3,10);
plot( t, VT);
title('V_T');
grid on;
subplot(6,3,11);
plot( t, h);
title('h');
grid on;
subplot(6,3,12);
plot( t, P);
title('Engine Power, P');
grid on;

subplot(6,3,13);
plot( t, U(1,:)*180/pi);
title('\delta_a');
grid on;
subplot(6,3,14);
plot( t, U(2,:)*180/pi);
title('\delta_e');
grid on;
subplot(6,3,15);
plot( t, U(3,:)*180/pi);
title('\delta_r');
grid on;

subplot(6,3,16);
plot( t, atan(w./u)*180/pi);
title('\alpha');
grid on;
subplot(6,3,17);
plot( t, asin(v./VT)*180/pi);
title('\beta');
grid on;

%%
if show3d
    figure(2); clf;
    ax1=subplot(121);
    hold on;
    plot3(X,Y,h,'rs')
    scatter3( x_traj(1,:), x_traj(2,:), x_traj(3,:), 'k.')
    view(2)
    ylabel('x (North)');
    xlabel('y (East)');
    zlabel('h (Up)');
    grid on;
    axis equal
    
    ax2=subplot(122);
    hold on;
    plot(h,Y,'rs')
    scatter( x_traj(3,:),x_traj(2,:), 'k.')
    ylabel('x (North)');
    xlabel('h (Up)');
    grid on;
    axis equal
    
    linkaxes([ax1 ax2], 'y')
end

%%
% figure(2); 
% hold on;
% scatter3( Y(1:10),X(1:10),h(1:10),'rs')
% scatter3( x_traj(2,1:10), x_traj(1,1:10), x_traj(3,1:10), 'k.')
% view(2)
% ylabel('x (North)');
% xlabel('y (East)');
% zlabel('h (Up)');
% grid on;
% axis equal