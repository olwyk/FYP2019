function Z = updateInformationStates(dmpc,k,X,x_ref,x_ref_assignments)

Na = dmpc.Na;
Np = dmpc.Np;
R = dmpc.maxRefRadius;
Z = dmpc.Z;

for i=1:Na
    refIdx = x_ref_assignments( i);
    r = norm( X(1:2,k,i) - x_ref(1:2,refIdx));
    if r > R
        Z(1:2,k:k+Np,i) = R*( x_ref(1:2,refIdx) - X(1:2,k,i))/r *ones(1,Np+1);
        Z(3,k:k+Np,i) = x_ref(3,refIdx) *ones(1,Np+1);
    else
        Z(1:3,k:k+Np,i) = x_ref(:,refIdx) *ones(1,Np+1);
    end
end


end