function [N,Nv] = findNeighbouringAgents(dmpc,k,i,X)

Na = dmpc.Na;
Np = dmpc.Np;
R = dmpc.commRadius;
Rv = dmpc.detRadius;
V = dmpc.V;
Nava = dmpc.Nava;
w = dmpc.virtualAgentWidth;
Ndo = dmpc.Ndo;

%%% Real Agents
% Update the matrix of neighbouring agents for the current agent, i
% For each step forward into the prediction horizon
% Note that agents assume their communication doesn't change over the
% prediction horizon
N = cell(1,Np);
allAgentIdxs = 1:Na;
allAgentIdxsExcept = allAgentIdxs( allAgentIdxs~=i);
% Find the distances to each other agent
agentDistances = vecnorm( X(1:3,k,allAgentIdxsExcept) - ...
    X(1:3,k,i), 2, 1);
% Determine which agents are neighbours using the communication radius
N_k = allAgentIdxsExcept( agentDistances <= R);
% Store the vector of current neighbours to agent i
for p=1:Np
    % This is intentional - you can't predict that you will enter
    % communication range
    N{p} = N_k;
end


%%% Virtual Agents
if Ndo > 0
    % Find the neighbouring virtual agents
    Nv = cell(1,Np);
    allVirtualAgents = 1:size(V,2);
    % Find the distances to each other agent
    agentDistances = vecnorm( V(1:3,allVirtualAgents) - ...
        X(1:3,k,i), 2, 1);
    % Sort by ascending distance
    [r,idx] = sort( agentDistances);
    % Assign virtual agent indices to each distance
    A = [reshape(idx,[],1) reshape(r,[],1)];
    % Remove all agents outside the cutoff distance
    rCutoffIdx = r<=Rv;
    A = A( rCutoffIdx, :);
    % Find the angles to each agent
    x = V(1,idx); x = x( rCutoffIdx);
    y = V(2,idx); y = y( rCutoffIdx);
    z = V(3,idx); z = z( rCutoffIdx);
    % azimuth : angle anticlockwise from x
    % inclination : angle from x-y plane
    azimuth = atan2( y-X(2,k,i),x-X(1,k,i));
    inclination = atan2( z-X(3,k,i), r( rCutoffIdx));
    A = [A azimuth' inclination']; % [index r psi theta]
    B = []; % [index r psi theta]
    for g=1:size(A,1)
        if ~isempty( B)
            gamma_all = 0.5*max( [ abs( (w*cos(B(:,3)))./B(:,2)), ...
                                   abs( (w*cos(B(:,4)))./B(:,2))], [], 2);
        end
        shouldAddPt = true;
        for h=1:size(B,1)
            gamma = gamma_all(h);
            if abs( A(g,3) - B(h,3)) < gamma && abs( A(g,4) - B(h,4)) < gamma
                shouldAddPt = false;
                break;
            end
        end
        if shouldAddPt
            B = [B; A(g,:)];
        end
        if size( B,1) >= Nava
            break;
        end
    end
    if size(B,1) > 0
        N_k = B(:,1);
    else
        N_k = [];
    end
else
    N_k = [];
end

% Store the vector of current neighbours to agent i
for p=1:Np
    % This is intentional - you can't predict that you will enter
    % detection range
    Nv{p} = N_k;
end


end