function J = f16_objFunc(y,traj,Np)

Q = eye(3);
R = eye(4) * (1.7e3)^2;

J = 0;
for k=1:Np
    x = y(7:9,k);
    u = y(14:17,k);
    xr = traj(1:3,k);
    
    J = J + (x-xr)'*Q*(x-xr) + u'*R*u;
end
    
end