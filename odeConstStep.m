function [tOut,xOut] = odeConstStep( odefn, T, x0)

% ASSUMPTION: T(1) = 0;

N = 5;
dt = ( T(2)-T(1)) / N;

tOut = ( 0:dt:T(end))';
xOut = zeros( size(x0,1), length(tOut));
xOut(:,1) = x0;

for i=2:length(tOut)
    t = tOut(i);
    xOut(:,i-1)
    xOut(:,i) = xOut(:,i-1) + dt*odefn(t,xOut(:,i-1));
end

tOut = tOut(1:N:end);
xOut = xOut(:,1:N:end);


end