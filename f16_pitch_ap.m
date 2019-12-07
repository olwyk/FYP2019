function dele = f16_pitch_ap(k,x,theta_c,theta_err_sum,h_c,h_err_sum)
% Gains
K1 = 50;
K2 = 10;
K3 = 1;
K4 = 0.01;

% Variables
theta = x(11);
q = x(5);
h = x(9);

% Reduce tendency to fall below 2 km altitude
h_c = max( 2e3, h_c);

% Control law
dele = K1*(theta - theta_c) + K2*q + K3*theta_err_sum + K4*( h - h_c);

% Saturate
dele = min( max( dele, -21.5*pi/180), +21.5*pi/180);

end