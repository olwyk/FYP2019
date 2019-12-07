function [opt_cost] = optFn( dmpc,k,i,X,U,x_ref,x_opt,u_opt)
% dmpc: Struct containing problem-specific constants
% k : Current time instant
% i : Current agent
% X : All state vectors
% U : All control vectors
% x_ref : Reference state vectors
% x_opt: Optimal state sequence (used as the optimisation variable)
% u_opt: Optimal input sequence (used as the optimisation variable)

% Extract data from the dmpc struct
Np = dmpc.Np;

% Add the current agent's trial state sequence to the matrix
% - Dim. 1: State variables
% - Dim. 2: Time instants
% - Dim. 3: Agents
X(:,k+1:k+Np,i) = x_opt;

% Add the current agent's trial control inputs to the matrix
% - Dim. 1: State variables
% - Dim. 2: Time instants
% - Dim. 3: Agents
U(:,k:k+Np-1,i) = u_opt;

% Compute the sequence of predicted adjacency matrices
[N,Nv] = findNeighbouringAgents(dmpc,k,i,X);

% Evaluate the cost function for this control sequence
opt_cost = J(dmpc,k,i,X,U,N,x_ref,Nv);

end