function y0 = f16_init(k,X_f16,u0,Np,dt)

for p=1:Np
    X_f16(:,k+p) = X_f16(:,k+p-1) + dt*f16_dynamics( X_f16(:,k+p-1), u0(:,p));
end
y0 = [X_f16(:,k+1:k+Np); u0];
y0 = reshape( y0, [], 1);

end