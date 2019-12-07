function xd = f16_dynamics(x,u)

% Constants (in kg-m-s)
m = 9298.6;
b = 9.144;
mac = 3.4503;
S = 27.8709;
Jxx = 12820.61;
Jyy = 75673.62;
Jzz = 85552.11;
Jxz = 1331.41;
[~,a,~,rho] = atmosisa( -x(9));
g = 9.81;
xcgref = 0.35*mac;
xcg = 0.25*mac;

% Control variables (with saturation)
dela = min( max( u(1), -25.0*pi/180), +25.0*pi/180);
dele = min( max( u(2), -21.5*pi/180), +21.5*pi/180);
delr = min( max( u(3), -30.0*pi/180), +30.0*pi/180);
delt = min( max( u(4), 0.0), 1.0);

% State variables
u = x(1);
v = x(2);
w = x(3);
p = x(4);
q = x(5);
r = x(6);
X = x(7);
Y = x(8);
h = x(9);
phi = x(10);
theta = x(11);
psi = x(12);
P = x(13);

% Derived variables
VT = norm([u v w]);
alpha = max( -10*pi/180, min( atan(w/u), 45*pi/180));
beta = max( -30*pi/180, min( asin(v/VT), 30*pi/180));
Phat = p*b/2/VT;
Qhat = q*mac/2/VT;
Rhat = r*b/2/VT;
qbar = 0.5*rho*VT^2;
Mach = VT/a;

% Aerodynamic coefficients - components
Cx = [-1.943367e-2 2.136104e-1 -2.903457e-1 -3.348641e-3 -2.060504e-1 ...
      +6.988016e-1 -9.035381e-1] * ...
      [1; alpha; dele^2; dele; alpha*dele; alpha^2; alpha^3];
Cxq = [+4.833383e-1 +8.644627e0 +1.131098e1 -7.422961e1 +6.075776e1] * ...
    [1; alpha; alpha^2; alpha^3; alpha^4];
Cy = [-1.145916e0 +6.016057e-2 +1.642479e-1] * ...
    [beta; dela; delr];
Cyp = [-1.006733e-1 +8.679799e-1 +4.260586e0 -6.923267e0] * ...
    [1; alpha; alpha^2; alpha^3];
Cyr = [+8.071648e-1 +1.189633e-1 +4.177702e0 -9.162236e0] * ...
    [1; alpha; alpha^2; alpha^3];
Cz = (1-beta^2).*[-1.378278e-1 -4.211369e0 +4.775187e0 -1.026225e1 ...
                  +8.399763e0] * ...
     [1; alpha; alpha^2; alpha^3; alpha^4] + -4.354000e-1*dele;
Czq = [-3.054956e1 -4.132305e1 +3.292788e2 -6.848048e2 +4.080244e2] * ...
    [1; alpha; alpha^2; alpha^3; alpha^4];
Cl = [-1.058583e-1 -5.776677e-1 -1.672435e-2 +1.357256e-1 +2.172952e-1 ...
      +3.464156e0 -2.835451e0 01.098104e0] * ...
    [beta; alpha*beta; alpha^2*beta; beta^2; alpha*beta^2; alpha^3*beta; ...
     alpha^4*beta; alpha^2*beta^2];
Clp = [-4.126806e-1 -1.189974e-1 1.247721e0 -7.391132e-1] * ...
    [1; alpha; alpha^2; alpha^3];
Clr = [+6.250437e-2 +6.067723e-1 -1.191964e0 +9.100087e0 -1.192672e1] * ...
    [1; alpha; alpha^2; alpha^3; alpha^4];
Clda = [-1.463144e-1 -4.073901e-2 +3.253159e-2 +4.851209e-1 +2.978850e-1 ...
        -3.746393e-1 -3.213068e-1] * ...
    [1; alpha; beta; alpha^2; alpha*beta; alpha^2*beta; alpha^3];
Cldr = [+2.635729e-2 -2.192910e-2 -3.152901e-3 -5.817803e-2 +4.516159e-1 ...
        -4.928702e-1 -1.579864e-2] * ...
    [1; alpha; beta; alpha*beta; alpha^2*beta; alpha^3*beta; beta^2];
Cm = [-2.029370e-2 +4.660702e-2 -6.012308e-1 -8.062977e-2 +8.320429e-2 ...
      +5.018538e-1 +6.378864e-1 +4.226356e-1] * ...
    [1; alpha; dele; alpha*dele; dele^2; alpha^2*dele; dele^3; alpha*dele^2];
Cmq = [-5.159153e0 -3.554716e0 -3.598636e1 +2.247355e2 -4.120991e2 ...
       +2.411750e2] * ...
    [1; alpha; alpha^2; alpha^3; alpha^4; alpha^5];
Cn = [+2.993363e-1 +6.594004e-2 -2.003125e-1 -6.233977e-2 -2.107885e0 ...
      +2.141420e0 8.476901e01] * ...
    [beta; alpha*beta; beta^2; alpha*beta^2; alpha^2*beta; ...
     alpha^2*beta^2; alpha^3*beta];
Cnp = [+2.677652e-2 -3.298246e-1 +1.926178e-1 +4.013325e0 -4.404302e0] * ...
    [1; alpha; alpha^2; alpha^3; alpha^4];
Cnr = [-3.698756e-1 -1.167551e-1 -7.641297e-1] * ...
    [1; alpha; alpha^2];
Cnda = [-3.348717e-2 +4.276655e-2 +6.573646e-3 +3.535831e-1 -1.373308e0 ...
        1.237582e0 2.302543e-1 -2.512876e-1 +1.588105e-1 -5.199526e-1] * ...
    [1; alpha; beta; alpha*beta; alpha^2*beta; alpha^3*beta; alpha^2; ...
     alpha^3; beta^3; alpha*beta^3];
Cndr = [-8.115894e-2 -1.156580e-2 +2.514167e-2 +2.038748e-1 -3.337476e-1 ...
        +1.004297e-1] * ...
    [1; alpha; beta; alpha*beta; alpha^2*beta; alpha^2];

% Aerodynamic coefficients (nondimensional)
Cx = Cx + Cxq*Qhat;
Cy = Cy + Cyp*Phat + Cyr*Rhat;
Cz = Cz + Czq*Qhat;
Cl = Cl + Clp*Phat + Clr*Rhat + Clda*dela + Cldr*delr;
Cm = Cm + Cmq*Qhat + Cz*(xcgref - xcg);
Cn = Cn + Cnp*Phat + Cnr*Rhat + Cnda*dela + Cndr*delr - Cy*(xcgref - xcg)*mac/b;

% Aerodynamic forces and moments
Xa = qbar*S*Cx;
Ya = qbar*S*Cy;
Za = qbar*S*Cz;
L = qbar*S*b*Cl;
M = qbar*S*mac*Cm;
N = qbar*S*b*Cn;

% Thrust
XT = f16_engine(delt,h,Mach);

% Engine power (first-order lag)
% Commanded power
Pc = f16_engine_thrust_gear( delt);
if Pc >= 50
    if P >= 50
        T = 5;
        P2 = Pc;
    else
        P2 = 60;
        T = f16_engine_time_constant( P2-P);
    end
else
    if P >= 50
        T = 5;
        P2 = 40;
    else
        P2 = Pc;
        T = f16_engine_time_constant( P2-P);
    end
end
% Derivative of engine power
Pd = T*(P2-P); 

% All state derivatives
xd(1) = r*v - q*w - g*sin(theta) + (Xa+XT)/m;
xd(2) = -r*u + p*w + g*sin(phi)*cos(theta) + Ya/m;
xd(3) = q*u - p*v + g*cos(phi)*cos(theta) + Za/m;
% xd(4) = 1/(Jxx*Jzz-Jxz^2)*( Jxz*(Jxx-Jyy+Jzz)*p*q - ( Jzz*(Jzz-Jyy)+Jxz^2)*q*r + Jzz*L + Jxz*N);
% xd(5) = 1/Jyy*( (Jzz-Jxx)*p*r - Jxz*(p^2-r^2) + M);
% xd(6) = 1/(Jxx*Jzz-Jxz^2)*( ( (Jxx-Jyy)*Jxx + Jxz^2)*p*q - Jxz*(Jxx-Jyy+Jzz)*q*r + Jxz*L + Jxx*N);
xd(4) = 1/Jxx*( (Jyy-Jzz)*q*r + Jxz*p*q + L);
xd(5) = 1/Jyy*( (Jzz-Jxx)*r*p + Jxz*(r^2-p^2) + M);
xd(6) = 1/Jzz*( (Jxx-Jyy)*p*q - Jxz*q*r + N);
xd(7) = u*cos(theta)*cos(psi) + v*( -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)) + ...
        w*( sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
xd(8) = u*cos(theta)*sin(psi) + v*( cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)) + ...
        w*( -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));
xd(9) = u*sin(theta) - v*sin(phi)*cos(theta) - w*cos(phi)*cos(theta);
xd(10) = p + tan(theta)*( q*sin(phi) + r*cos(phi));
xd(11) = q*cos(phi) - r*sin(phi);
xd(12) = 1/cos(theta)*( q*sin(phi) + r*cos(phi));
xd(13) = Pd;

xd = reshape( xd, 13, 1);

end