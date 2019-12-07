function xhat = initAircraftStates(dmpc,X,VT)

Np = dmpc.Np;
Na = dmpc.Na;
k_max = dmpc.k_max;


xhat = zeros(13,k_max+Np,Na);
for i=1:Na
    xhat(:,:,i) = repmat( [VT;0;0; 0;0;0; X(:,1,i); 20], 1, k_max+Np);
    tmp = xhat(12,:,i);
    xhat(12,:,i) = xhat(10,:,i);
    xhat(10,:,i) = tmp;
    xhat(9,:,i) = -xhat(9,:,i);
end

end