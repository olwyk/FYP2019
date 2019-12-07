function dmpc = updateHistory(dmpc,k,X)

Na = dmpc.Na;

detectedVirtualAgents = [];
for i=1:Na
    [N,Nv] = findNeighbouringAgents(dmpc,k,i,X);
    
    dmpc.history.Adj(i,N{1},k) = ones( 1, length( N{1}));
    dmpc.history.Adj(N{1},i,k) = dmpc.history.Adj(i,N{1},k)';
    
    detectedVirtualAgents = [detectedVirtualAgents; Nv{1}];
end
dmpc.history.detectedVirtualAgents{k} = unique( detectedVirtualAgents);


end