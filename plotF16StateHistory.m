function [] = plotF16StateHistory(figH1,figH2,dmpc,y,U,i)

fontSizeNum = 18;

dt = dmpc.dt;
k_max = dmpc.k_max;

t = 0:dt:(k_max-1)*dt;

u = y(1,:,i);
v = y(2,:,i);
w = y(3,:,i);
p = y(4,:,i);
q = y(5,:,i);
r = y(6,:,i);
X = y(7,:,i);
Y = y(8,:,i);
h = y(9,:,i);
phi = y(10,:,i);
theta = y(11,:,i);
psi = y(12,:,i);
P = y(13,:,i);

VT = vecnorm([u;v;w]);

figure( figH1); clf;
subplot(3,3,1);
plot( t, u, 'LineWidth', 1.5);
title('Body X Velocity, $U$ [m/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,2);
plot( t, v, 'LineWidth', 1.5);
title('Body Y Velocity, $V$ [m/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,3);
plot( t, w, 'LineWidth', 1.5);
title('Body Z Velocity, $W$ [m/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

subplot(3,3,4);
plot( t, phi*180/pi, 'LineWidth', 1.5);
title('Roll, $\phi$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,5);
plot( t, theta*180/pi, 'LineWidth', 1.5);
title('Pitch, $\theta$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,6);
plot( t, psi*180/pi, 'LineWidth', 1.5);
title('Yaw, $\psi$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

subplot(3,3,7);
plot( t, p*180/pi, 'LineWidth', 1.5);
title('Body Roll Rate, $P$ [deg/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,8);
plot( t, q*180/pi, 'LineWidth', 1.5);
title('Body Pitch Rate, $Q$ [deg/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,9);
plot( t, r*180/pi, 'LineWidth', 1.5);
title('Body Yaw Rate, $R$ [deg/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

figure( figH2); clf;
subplot(3,3,1);
plot( t, VT, 'LineWidth', 1.5);
title('Airspeed, $V_T$ [m/s]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,2);
plot( t, h, 'LineWidth', 1.5);
title('Altitude, $h$ [m]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,3);
plot( t, P, 'LineWidth', 1.5);
title('Engine Power, $P_e$ [\%]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

subplot(3,3,4);
plot( t, U(1,:,1)*180/pi, 'LineWidth', 1.5);
title('Aileron, $\delta_a$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,5);
plot( t, U(2,:,1)*180/pi, 'LineWidth', 1.5);
title('Elevator, $\delta_e$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,6);
plot( t, U(3,:,1)*180/pi, 'LineWidth', 1.5);
title('Rudder, $\delta_r$ [deg]', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

subplot(3,3,7);
plot( t, U(4,:,1), 'LineWidth', 1.5);
title('Throttle, $\delta_t$', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,8);
plot( t, atan(w./u)*180/pi, 'LineWidth', 1.5);
title('Angle of Attack, $\alpha [deg]$', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;
subplot(3,3,9);
plot( t, asin(v./VT)*180/pi, 'LineWidth', 1.5);
title('Sideslip, $\beta [deg]$', 'interpreter', 'latex');
set(gca,'FontSize',fontSizeNum);
grid on;

end