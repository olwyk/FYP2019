function [xd,u] = f16_odefn(dmpc,t,x,x_traj,VT_c)

% x: [u v w p q r pN pE h phi theta psi]
% x_traj: [pN pE h phi theta psi]

% Find the current k for tracking
dt = dmpc.dt;
k = 1+floor(t/dt);
if k == size( x_traj, 2) + 1
    k = k - 1;
end

% Set the current trajectory target
x_traj_k = x_traj(:,k);

% Navigation
h_c = x_traj_k(3);
phi_c = x_traj_k(4);
theta_c = x_traj_k(5);
psi_c = x_traj_k(6);
h = x(9);
phi = x(10);
theta = x(11);
psi = x(12);

% Accumulated error
persistent err_history;
n_int = 100;
if isempty(err_history)
    err_history = zeros(n_int,4);
end
err_history(1:n_int-1,:) = err_history(2:n_int,:);
err_history(n_int,:) = [phi-phi_c theta-theta_c psi-psi_c h-h_c];
phi_err_sum = sum( err_history(:,1));
theta_err_sum = sum( err_history(:,2));
psi_err_sum = sum( err_history(:,3));
h_err_sum = sum( err_history(:,4));

% Pitch
dele = f16_pitch_ap(k,x,theta_c,theta_err_sum,h_c,h_err_sum);

% Heading
dela = f16_heading_ap(k,x,psi_c,phi_c,psi_err_sum,phi_err_sum);

% Thrust
delt = f16_thrust_ap(x,VT_c);

% Sideslip damping
delr = f16_sideslipdamper_ap(x,psi_c,psi_err_sum);

% Form control input
u = [dela; dele; delr; delt];

% Dynamics
xd = f16_dynamics(x,u);

end