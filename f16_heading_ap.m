function dela = f16_heading_ap(k,x, psi_c, phi_c, psi_err_sum, phi_err_sum)
% Gains
K1 = 0.4;%0.6;
K2 = 0.1;%0.5;
K3 = 0.001;%0.2;

% Variables
p = x(4);
phi = x(10);

% Control law
% dela = K1*( psi - psi_c) + K2*p + K3*( phi - phi_c) + K4*psi_err_sum;
dela = K1*( phi - phi_c) + K2*p + K3*phi_err_sum;

% Saturation 
dela = min( max( dela, -25.0*pi/180), +25.0*pi/180);

end