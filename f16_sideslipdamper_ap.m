function delr = f16_sideslipdamper_ap(x,psi_c,psi_err_sum)

K1 = 0.3;%3;%0.3;
K2 = 0.01;%0.005;%0.01;
K3 = 10;%10;

% Variables
u = x(1);
v = x(2);
w = x(3);
VT = norm([u v w]);
beta = asin(v/VT);
psi = x(12);

% Control law
% delr = -K1*beta;
delr = K1*( psi - psi_c) + K2*psi_err_sum - K3*beta;

% Saturate
delr = min( max( delr, -30.0*pi/180), +30.0*pi/180);

end