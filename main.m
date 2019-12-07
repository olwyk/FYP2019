%% Nomenclature
%{

i     : Agent enumeration
j     : Neighbour enumeration
k     : Time instant 
l     : Number of time steps beyond k
Na    : Number of agents
Np    : Number of prediction time steps (state horizon)
Nc    : Control horizon
No    : Number of obstacles
n     : Number of state variables
m     : Number of control variables
x     : State variable
u     : Control variable
ulim  : Control variable limits
Q     : State trajectory tracking weight matrix
R     : Control input weight matrix
S     : Formation weight matrix
T     : Terminal condition weight matrix
L     : Interaction Laplacian matrix
J(.)  : Cost function
N(i)  : Set of neighbours of agent i
f(x,u): Agent dynamics
V     : Virtual agent states

%}

%% User Options
saveVideo = false;
useActiveConstraints = false;
useF16Kinematics = true;
showF16Plots = false;

% Cost function tuning
agentStandoff = 1e3;
obstacleStandoff = 2e3;
referenceStandoff = 5e3; % <- check with VT^2/g/tan(phi)
Jq  = 1e4 /referenceStandoff^2;
Jr  = 1e4;
Js  = (1e-4 -eps)/agentStandoff^2;
Jsv = (1e-4 -eps) /obstacleStandoff^2/1; % <- check that this is Ndo
Jt = 0*1e4;

if ~useActiveConstraints
    fprintf('Warning: Active constraint method disabled.\n');
end


%% Obstacle Definitions
%%% Fixed obstacles
fixedObstacleCentres = [];
fixedObstacleAxes = [];

%%% Detected obstacles
% [[centre] [axes]]
detectedObstacles = [];
detectedObstacles{1} = [[15e3;0e3;-10e3] [2e3;20e3;10e3]];
detectedObstacles{2} = [[0e3;22e3;-10e3] [15e3;2e3;10e3]];
detectedObstacles{3} = [[0e3;-22e3;-10e3] [15e3;2e3;10e3]];
virtualAgentWidth = 100;


%% Constants
% Maximum number of time iterations
k_max = 400;
% Time step 
dt = 1.0;
% Time step for F-16 autopilots
dt_f16 = dt/100;
% Number of agents
Na = 1;
% Prediction horizon (Np-Nc should be large)
Np = 40;%25; 
% Control horizon (Nc should be very small)
Nc = 1;%3;
% Number of solver iterations per active constraint loop
Ns = 50;
% Number of state variables
n = 6;
% Number of control variables
m = 2;
% Communication radius
commRadius = 10e3;
% Detection radius
detRadius = 10e3;
% Collision radius; the distance within which agents are considered to have
% collided
collisionRadius = 15;
% Maximum distance to the reference point (used to prevent collisions)
maxRefRadius = 100e3;
% Number of virtual agents dealt with at each instant
numActiveVirtualAgents = 100;
% Constant true airspeed (M=0.7)
VT = 250;
% Gravitational constant
g = 9.81;
% Epsilon (used to prevent collisions)
epsln = eps;
% Constraint epsilon (distance either side of a hard obstacle considered a collision)
constraintEpsln = 10;%0.25;
% State weighting matrix
Q = eye(3);
% Q(3,3) = 1e3;
% Input weighting matrix
R = eye(m);
% Formation weighting matrix
S = eye(3);
% Dynamic obstacle weighting matrix
Sv = eye(3);
% Terminal condition weighting matrix
T = eye(3);
% Store all constants in a struct
dmpc.k_max = k_max;
dmpc.dt = dt;
dmpc.Na = Na;
dmpc.Np = Np;
dmpc.Nc = Nc;
dmpc.Ns = Ns;
dmpc.No = size( fixedObstacleCentres,2);
dmpc.Ndo = length( detectedObstacles);
dmpc.n = n;
dmpc.m = m;
dmpc.commRadius = commRadius;
dmpc.detRadius = detRadius;
dmpc.collisionRadius = collisionRadius;
dmpc.maxRefRadius = maxRefRadius;
dmpc.Nava = numActiveVirtualAgents;
dmpc.VT = VT;
dmpc.g = 9.81;
dmpc.epsln = epsln;
dmpc.constraintEpsln = constraintEpsln;
dmpc.Q = Q*Jq;
dmpc.R = R*Jr;
dmpc.S = S*Js;
dmpc.Sv = Sv*Jsv;
dmpc.T = T*Jt*Np;%T*1*Np*0; % <- !!
dmpc.virtualAgentWidth = virtualAgentWidth;
dmpc.fixedObstacleCentres = fixedObstacleCentres;
dmpc.fixedObstacleAxes = fixedObstacleAxes;
dmpc.useActiveConstraints = useActiveConstraints;
dmpc.useF16Kinematics = useF16Kinematics;
dmpc.P = zeros(k_max,6,Np,Na);
for i=1:Na
    dmpc.J{i} = [];
end


%% Initialisation
% Storage of all agent states
% - Dim. 1: State variables
% - Dim. 2: Time instants
% - Dim. 3: Agents
X =  zeros( n, k_max+Np, Na);
% Storage of all agent inputs
U =  zeros( m, k_max+Np, Na);
% Reference point(s)  
x_ref = [ 25e3;
          0e3;
         -5e3];
x_ref_assignments = ones(1,Na);
if length( x_ref_assignments) ~= Na
    error('Check x_ref_assignments');
end
% State limits
xlim = [-Inf  +Inf;    % x
        -Inf  +Inf;    % y
        -Inf  +0;      % z
        -Inf  +Inf;    % yaw
        -pi/16 +pi/16;   % flight-path angle
        -pi/4 +pi/4];  % roll
% Control limits
ulim = [-.087 +.087;   % pitch angle rate [rad/s]; -5 deg/s, +5 deg/s
        -1.57 +1.57];  % roll rate [rad/s]; -90 deg/s, +90 deg/s
% Randomise initial agent positions
X(:,1,:) = [-20e3;   % x
            0e3;   % y
            -5e3*ones(1,Na);% z
            zeros(1,Na);         % yaw
            zeros(1,Na);         % pitch
            zeros(1,Na)];        % roll
% Add path for IPOPT
addpath('./ipopt_lib');
% Clear the history
dmpc.history.Adj = [];
dmpc.history.detectedVirtualAgents = [];
dmpc.history.objVal = zeros( k_max,Na);
% Initialise the information states
dmpc.Z = zeros(3,k_max+Np,Na);
dmpc.Z = updateInformationStates(dmpc,1,X,x_ref,x_ref_assignments);
% Generate the virtual agents
% Axis 1: State variables (position only)
% Axis 2: Different points defining the dynamic obstacle
[dmpc.V,dmpc.detVertices] = generateVirtualAgentStates( dmpc, detectedObstacles, virtualAgentWidth);
% Aircraft states and controls
if useF16Kinematics
    xhat = initAircraftStates(dmpc,X,VT);
    uhat = zeros(4,k_max+Np,Na);
    dmpc.xhat_all = zeros( 13, (k_max+Np)*(dt/dt_f16), Na);
    dmpc.uhat_all = zeros( 4, (k_max+Np)*(dt/dt_f16), Na);
    % Clear persistent variables
    clear('f16_odefn');
else
    xhat = [];
    uhat = [];
    dmpc.xhat = [];
end


%% Computations
fprintf('Solving...\n');
tic;

%%% INITIALISATION
% Apply a feasible control sequence to initialise the agents
u0 = zeros(m,Np);
% For each agent i
k = 1;
for i=1:Na
    initStates( dmpc, k, i, X, U, u0);                    
end

%%% MAIN LOOP
waitbar_handle = waitbar(0, 'Solving...');
% At each time instant k
X_new = X;
k = 1;                                                    
while k <= k_max
    % For each agent i
    for i=1:Na
        %%% Find the optimal control sequence for this agent

        % Wrapper for the gradient function
        [N,Nv] = findNeighbouringAgents(dmpc,k,i,X_new);
        gradientFnWrapper = @(y) gradientFn( dmpc,k,i,X_new,...
            N, Nv,...
            x_ref,...
            reshape( y(1:n*Np), n, Np), ...
            reshape( y(n*Np+1:end), m, Np)); 

        % Optimisation variable bounds
        lb = [repmat( xlim(:,1), Np, 1);
              repmat( ulim(:,1), Np, 1)];
        ub = [repmat( xlim(:,2), Np, 1);
              repmat( ulim(:,2), Np, 1)];

        % Optimisation uses the following form to store variables
        % y: [x_opt_1; ... ; x_opt_n; u_opt_1; ...; u_opt_n]
        optFnWrapper = @(y) optFn( dmpc,k,i,X_new,U,x_ref,...
            reshape( y(1:n*Np), n, Np), ...
            reshape( y(n*Np+1:end), m, Np)); 

        % Initialise with no active inequality constraints
        if useActiveConstraints
            Qi = [];
            Qi_prev = [];
        else
            Qi = [1:Np*dmpc.No]';
        end
        info_prev = [];

        % Use active constraints to accelerate the solver
        continueConstraintsLoop = true;
        constraintsLoopIdx = 1;
        % Initial Lagrange multipliers
        lambda = zeros(Np*n+Np*dmpc.No,1);
        while continueConstraintsLoop
            % Impose optimisation constraints
            optConstraintsWrapper = @(y) optConstraints( dmpc, ...
                reshape( y(1:n*Np), n, Np), ...
                reshape( y(n*Np+1:end), m, Np), ...
                X(:,k,i), Qi);

            % Feasible initial condition
%             u0 = zeros(m,Np);
            u0 = U(:,k:k+Np-1,i); % use the previous predicted inputs
            x0 = initStates( dmpc, k, i, X, U, u0);
%             x0 = X(:,k+1:k+Np,i);
            y0 = [reshape(x0,[],1); reshape(u0,[],1)]; 

            % Solve the optimisation problem
            constraintsJacobianWrapper = @(y) constraintsJacobian( dmpc, y, Qi);
            constraintsJacobianStructWrapper = @() constraintsJacobianStructure( dmpc, Qi);
            hessianWrapper = @(x,sigma,lambda) hessian( dmpc,k,x,N,Nv,X_new,sigma,lambda,Qi);
            hessianStructureWrapper = @() hessianStructure( dmpc);

            [y,info] = ipoptCall( dmpc, optFnWrapper, y0, lb, ub, gradientFnWrapper, ...
                        optConstraintsWrapper, constraintsJacobianWrapper, ...
                        constraintsJacobianStructWrapper, ...
                        hessianWrapper, hessianStructureWrapper, ....
                        Qi_prev, Qi, info_prev);
            dmpc.J{i} = [dmpc.J{i}; info.objective];

            % Track the Lagrange multipliers for warm-starting IPOPT
            lambda = info.lambda;

            % Reshape the optimiser output
            x_opt = reshape( y(1:n*Np), n, Np);
            u_opt = reshape( y(n*Np+1:end), m, Np);

            % Continue if not using accelerated processing
            if ~useActiveConstraints
                break;
            end

            % Update the initial condition for the solver
            y0 = y;

            % Update the active constraints
            [Qi_new,collisionFound] = getActiveConstraintIndices(dmpc,x_opt);
            Qi_prev = Qi;
            Qi = unique( [Qi; Qi_new]);
            info_prev = info;

            % Check if a valid solution was found
            if info.status == 0 && collisionFound == false
                continueConstraintsLoop = false;
            elseif info.status < -1
                error('Solver failure.');
            elseif info.status > 0
                fprintf('Solver warning: %i\n', info.status);
                if info.status == 2
                    error('Infeasible problem detected.');
                end
            end
            dmpc.objVal(k,i) = info.objective/Np;

            % Track how many loops have been completed to detect a stall
            if constraintsLoopIdx > 10
                dmpc.Ns = dmpc.Ns + 50;
                if constraintsLoopIdx > 100
                    error('Infeasible problem detected by active set algorithm.');
                end
            end
            constraintsLoopIdx = constraintsLoopIdx + 1;
        end

        % Reset Ns
        dmpc.Ns = Ns;

        % Store the new sequence of predicted states
        X_new(:,k+1:k+Np,i) = x_opt;
        U(:,k:k+Np-1,i) = u_opt;
        for l=0:Nc-1
            dmpc.P(k+l,:,:,i) = x_opt;
        end

        % Pass this trajectory to the lower-level autopilots
        % Track over k+1:k+Nc
        % xhat: [u v w p q r x y h phi theta psi P]
        % NB: Different coordinate system adopted here to F-16 autopilot
        if useF16Kinematics
            x_traj = x_opt;
            x_traj(1,:) = x_opt(1,:); % x
            x_traj(2,:) = x_opt(2,:); % y
            x_traj(3,:) = -x_opt(3,:); % Z -> h
            x_traj(4,:) = x_opt(6,:); % roll
            x_traj(5,:) = x_opt(5,:); % pitch
            x_traj(6,:) = x_opt(4,:); % yaw
            xhat0 = xhat(:,k,i);
            f16_odefn_wrapper = @(t,xhat) f16_odefn( dmpc, t, xhat, x_traj, VT);
            % Nc+1 ensures at least 3 time points
            options = odeset('MaxStep', dt, 'InitialStep', dt_f16);
            [t,xhat_new,t_all,xhat_all] = solveRk4( f16_odefn_wrapper, [0:dt:dt*(Nc+1)], xhat0, options);
            xhat(:,k:k+Nc,i) = xhat_new(:,1:end-1);
            idx = 0;
            for t=[0:dt:dt*Nc]
                [~,uhat_new] = f16_odefn( dmpc, t, xhat(:,k+idx,i), x_traj, VT);
                uhat(:,k+idx,i) = uhat_new;
                idx = idx + 1;
            end
            dmpc.xhat_all(:,1+(k-1)*(dt/dt_f16):(k+Nc)*(dt/dt_f16)+1,i) = xhat_all;
        end

    end

    % Communicate the new sequence of predicted states
    X = X_new;  

    % Step forward by the control horizon
    % In effect, this applies Nc controls before recalculating
    k = k + Nc;

    % Reset the trajectory planner to the aircraft's location
    if useF16Kinematics
        for i=1:Na
            X(1:3,k,i) = xhat(7:9,k,i);
            X(3,k,i) = -X(3,k,i);
            % Don't set angles - this seems to destabilise the system
        end
    end

    % Update the information states
    dmpc.Z = updateInformationStates(dmpc,k,X,x_ref,x_ref_assignments);

    % Update history
    dmpc = updateHistory( dmpc, k, X);

    % Print progress
    waitbar( k/k_max, waitbar_handle);
end
close( waitbar_handle);
fprintf('Done.\n');
fprintf('Time elapsed: %.1f s\n', toc);

%%% Post-Processing
% Trim away the excess
X = X(:,1:k_max,:);
U = U(:,1:k_max,:);
if useF16Kinematics
    xhat = xhat(:,1:k_max,:);
    uhat = uhat(:,1:k_max,:);
    dmpc.xhat_all = dmpc.xhat_all(:,1:(k_max-1)*(dt/dt_f16),:);
end

% Convert to height
X(3,:,:) = -X(3,:,:);
x_ref(3,:) = -x_ref(3,:);

% Check for collisions
[numCollisions, minRadius, collisionK] = checkForCollisions( dmpc, X);
if numCollisions > 0
    fprintf('%i collisions detected.\n', numCollisions);
    collisionK'
end
if Na > 1
    fprintf('Minimum inter-agent distance: %.1f m\n', minRadius);
end


%% Display - animations
figH = figure(1);
set( figH, 'Position',[0 0 1500 800]);
minX1 = min([min(min(X(1,:,:))) min(x_ref(1,:))]);
maxX1 = max([max(max(X(1,:,:))) max(x_ref(1,:))]);
minX2 = min([min(min(X(2,:,:))) min(x_ref(2,:))]);
maxX2 = max([max(max(X(2,:,:))) max(x_ref(2,:))]);
minX3 = min([min(min(X(3,:,:))) min(x_ref(3,:))]);
maxX3 = max([max(max(X(3,:,:))) max(x_ref(3,:))]);
animOptions.viewLimits = [minX1 maxX1 minX2 maxX2 minX3 maxX3];
animOptions.is2D = false;
animOptions.trackView = false;
animOptions.showPts = true;
animOptions.freezeFrame = true;
animOptions.showK = k_max;
animOptions.trackCam = false;
animOptions.ptSize = 2;
animOptions.useF16Kinematics = useF16Kinematics;
animationFrames = animateSimulation(figH, dmpc, X, x_ref, xhat, animOptions);


%% Display - graphs
if useF16Kinematics && showF16Plots
    i=1;
    
    figure(3); clf;
    ax(1) = subplot(2,2,1); hold on;
    plot( dt*(0:k_max-1), X(6,:,i)*180/pi, 'k', 'LineWidth', 1.5);
    scatter( dt*(0:k_max-1), xhat(10,:,i)*180/pi, 20, 'r', 'filled');
%     scatter( 0:dt_f16:((k_max-1)*dt)-dt_f16, dmpc.xhat_all(10,:,1)*180/pi, 5, 'b', 'filled');
    legend('Trajectory', 'F-16', 'Location', 'NorthWest');
    xlabel('Time, t [s]', 'interpreter', 'latex');
    ylabel('Roll Angle, $\phi$ [deg]', 'interpreter', 'latex');
    grid on;
    set( gca, 'FontSize', 18);

    ax(2) = subplot(2,2,2); hold on;
    plot( dt*(0:k_max-1), X(5,:,i)*180/pi, 'k', 'LineWidth', 1.5);
    scatter( dt*(0:k_max-1), xhat(11,:,i)*180/pi, 20, 'r', 'filled');
%     scatter( 0:dt_f16:((k_max-1)*dt)-dt_f16, dmpc.xhat_all(11,:,1)*180/pi, 5, 'b', 'filled');
    legend('Trajectory', 'F-16', 'Location', 'NorthWest');
    xlabel('Time, t [s]', 'interpreter', 'latex');
    ylabel('Pitch Angle, $\theta$ [deg]', 'interpreter', 'latex');
    grid on;
    set( gca, 'FontSize', 18);

    ax(3) = subplot(2,2,3); hold on;
    plot( dt*(0:k_max-1), X(4,:,i)*180/pi, 'k', 'LineWidth', 1.5);
    scatter( dt*(0:k_max-1), xhat(12,:,i)*180/pi, 20, 'r', 'filled');
%     scatter( 0:dt_f16:((k_max-1)*dt)-dt_f16, dmpc.xhat_all(12,:,1)*180/pi, 5, 'b', 'filled');
    legend('Trajectory', 'F-16', 'Location', 'NorthWest');
    xlabel('Time, t [s]', 'interpreter', 'latex');
    ylabel('Yaw Angle, $\psi$ [deg]', 'interpreter', 'latex');
    grid on;
    set( gca, 'FontSize', 18);
    
    ax(4) = subplot(2,2,4); hold on;
    plot( dt*(0:k_max-1), X(3,:,i), 'k', 'LineWidth', 1.5);
    scatter( dt*(0:k_max-1), xhat(9,:,i), 20, 'r', 'filled');
%     scatter( 0:dt_f16:((k_max-1)*dt)-dt_f16, dmpc.xhat_all(9,:,1), 5, 'b', 'filled');
    legend('Trajectory', 'F-16', 'Location', 'NorthWest');
    xlabel('Time, t [s]', 'interpreter', 'latex');
    ylabel('Altitude, h [m]', 'interpreter', 'latex');
    grid on;
    set( gca, 'FontSize', 18);
    linkaxes(ax, 'x')
    
    figH1 = figure(4);clf;
    figH2 = figure(5);clf;
    plotF16StateHistory(figH1,figH2,dmpc,xhat,uhat,i);
end


% subplot(211);
% hold on;
% plot( uhat(1:3,1:k_max,1)'*180/pi);
% title('Control Inputs for Agent 1');
% legend('$\delta_a$', '$\delta_e$', '$\delta_r$', 'interpreter', 'latex', 'FontSize',16);
% 
% axis([1 k_max ylim]);
% subplot(212);
% hold on;
% plot( vecnorm([X(1,:,1)-x_ref(1,1); X(2,:,1)-x_ref(2,1)], 2), 'b');
% plot(-X(3,:,1), 'm');
% plot(-x_ref(3,1).*ones(1,k_max), 'm-^')
% title('State for Agent 1');
% axis([1 k_max ylim]);
% legend('R', 'h', 'h^r');


%% Animate F-16 with States
% figH = figure(4); clf;
% animateF16WithStates( figH, dmpc, i, xhat);


%% Save Video
if saveVideo
    fprintf('Saving video...');
    v = VideoWriter( ['Animations/' datestr( datetime, 'dd_mmm_HH_MM_SS')],'Motion JPEG AVI');
    v.Quality = 100;
    v.FrameRate = 20/dt;
    open(v);
    writeVideo(v,animationFrames);
    close(v);
    fprintf('Done.\n');
end
