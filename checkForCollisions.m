function [numCollisions,minRadius,collisionK] = checkForCollisions(dmpc,X)

k_max = dmpc.k_max;
Na = dmpc.Na;
Rc = dmpc.collisionRadius;

minRadius = Inf;
numCollisions = 0;
collisionK = [];
for k=1:k_max
    for i=1:Na
        for j=i+1:Na
            r = norm( X(1:3,k,i) - X(1:3,k,j));
            if r <= Rc
                numCollisions = numCollisions + 1;
                collisionK = [collisionK; k];
            end
            minRadius = min( minRadius, r);
        end
    end
end


end