% Trajectory
% X : [x y h psi theta phi]
% F-16
% Y : [U V W P Q R x y h phi theta psi Pe]


Np = 20;
Nc = 1;
k_max = k;
dt = 0.10;   % <- this probably needs to match

X_f16 = zeros(13,k_max+Np);
U_f16 = zeros(4,k_max+Np);
X_f16(:,1) = [250;0;0; 0;0;0; X(1,1);X(2,1);X(3,1); X(6,1);X(5,1);X(4,1); 20];
U_f16(:,1) = [0;-3.8*pi/180;0;0.3];

bounds = [150 250;     % U
          -Inf +Inf;   % V
          -Inf +Inf;   % W
          -Inf +Inf;   % P
          -Inf +Inf;   % Q 
          -Inf +Inf;   % R
          -Inf +Inf;   % x
          -Inf +Inf;   % y
          0 +Inf;      % h
          -pi/4 +pi/4; % phi
          -pi/4 +pi/4; % theta
          -Inf +Inf;   % psi
          0 100;       % Pe
          -25.0*pi/180 +25.0*pi/180; % Ail
          -21.5*pi/180 +21.5*pi/180; % Ele
          -30.0*pi/180 +30.0*pi/180; % Rud
          0.0 1.0]; % Thr
lb = repmat( bounds(:,1), Np, 1);
ub = repmat( bounds(:,2), Np, 1);

p = 1;
while p < k_max
    % Generate an MPC trajectory and accept the state sequence.
    % So long as the equality constraints are satisfied, the F-16
    % trajectory is valid.
    
    x_traj = X(:,p:p+Np,1);
    u0 = [U_f16(:,p:p+Np-1-Nc) repmat(U_f16(:,p+Np-1-Nc),1,Nc)];
    y0 = f16_init( p, X_f16, u0, Np, dt);
    
    f16_objFunc_wrapper = @(y) f16_objFunc( reshape( y, 17, Np), x_traj, Np);
    f16_nonlcon_wrapper = @(y) f16_nonlcon( reshape( y, 17, Np), X_f16(:,p), Np, Nc, dt);
    
    options = optimoptions('fmincon', 'Algorithm', 'interior-point', 'ConstraintTolerance', 1e-3, ...
        'MaxIterations', 1e3, 'MaxFunctionEvaluations', 1e4);%, 'PlotFcn', 'optimplotconstrviolation');
    [y,~,exitflag,output] = fmincon( f16_objFunc_wrapper, y0, [],[],[],[],lb,ub,f16_nonlcon_wrapper,options);
    
    y = reshape( y, 17, Np);
    X_f16(:,p+1:p+Np) = y(1:13,:);
    U_f16(:,p:p+Np-1) = y(14:17,:);
    
    p = p + Nc
end