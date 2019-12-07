function delt = f16_thrust_ap(x,VT_c)
% Tuning constants
Kp = 0.5;
delt_cruise = 0.5;

% Constants
VT = norm( x(1:3));

% Control law
delt = Kp*( VT_c - VT) + delt_cruise;

% Saturating
delt = min( max( delt, 0.0), 1.0);

end