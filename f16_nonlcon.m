function [c,ceq] = f16_nonlcon(y,xk,Np,Nc,dt)


c = [];

x_opt_pad = [xk y(1:13,:)];
u_opt_pad = [y(14:17,:) zeros(4,Np-Nc)];

ceq = zeros(13,Np);
for p=1:Np
    ceq(:,p) = ( x_opt_pad(:,p+1) - x_opt_pad(:,p))/dt - ... % finite difference approx.   
               ( f16_dynamics( x_opt_pad(:,p), u_opt_pad(:,p)));  % F-16 dynamics
end

end