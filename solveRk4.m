function [t,x1,t_all,x1_all] = solveRk4( odeFn, tSpan, x0, options)

dt = options.InitialStep;
n = size(x0,1);

% Constants
c2 = 1/3;
c3 = 2/3;
c4 = 1;
a21 = 1/3;
a31 = -1/3;
a32 = 1;
a41 = 1;
a42 = -1;
a43 = 1;
b1 = 1/8;
b2 = 3/8;
b3 = 3/8;
b4 = 1/8;

% Initialisation
t_all = 0:dt:tSpan(end);
x1_all = zeros( n, length(t_all));
x1_all(:,1) = x0;

% Integration loop
for i=2:length(t_all)
    t = t_all(i);
    x = x1_all(:,i-1);
    
    k1 = odeFn( t, x);
    k2 = odeFn( t + c2*dt, x + dt*( a21*k1));
    k3 = odeFn( t + c3*dt, x + dt*( a31*k1 + a32*k2));
    k4 = odeFn( t + c4*dt, x + dt*( a41*k1 + a42*k2 + a43*k3));
    
    x1_all(:,i) = x + dt*( b1*k1 + b2*k2 + b3*k3 + b4*k4);
end

% Extract points at tSpan
t = tSpan;
x1 = interp1( t_all, x1_all', t);
x1 = x1';

end