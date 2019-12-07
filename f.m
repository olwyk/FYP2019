function xd = f(dmpc,x,u)
% Agent dynamics function
% x  : Current state
% u  : Current control
% dt : Time step
% xd : State derivative

% x = [x y z psi gamma phi]

psi = x(4);
gamma = x(5);
phi = x(6);
g = dmpc.g;
VT = dmpc.VT;

xd = [VT*cos(gamma)*cos(psi);
      VT*cos(gamma)*sin(psi);
      -VT*sin(gamma);
      g/VT*tan(phi);
      u];


end