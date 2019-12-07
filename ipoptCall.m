function [u_opt,info] = ipoptCall( dmpc, optFn, u0, lb, ub, gradFn, consFn, ...
    consJacobian, consJacobianStruct, hessFn, hessFnStruct, ...
    Qi_prev, Qi, info_prev)

    n = dmpc.n;
    Np = dmpc.Np;
    
    % Debugging
    options.ipopt.print_level = 2;
%     options.ipopt.derivative_test = 'second-order';
%     options.ipopt.hessian_approximation = 'limited-memory';
    
    % Options
    options.lb = lb;
    options.ub = ub;
    options.ipopt.linear_solver    = 'mumps'; 
    options.ipopt.mu_strategy = 'adaptive';
    if dmpc.useActiveConstraints
        options.ipopt.max_iter = dmpc.Ns;
    else
        options.ipopt.max_iter = 1e3;
    end
%     options.ipopt.tol = 1e-3;  % default 1e-8
%     options.ipopt.constr_viol_tol = 1e-3;
    options.ipopt.print_frequency_iter = options.ipopt.max_iter;
    options.ipopt.point_perturbation_radius = 1e-10;
    options.cl = [zeros(n*Np,1); -Inf*ones( length(Qi),1)];
    options.cu = [zeros(n*Np,1); zeros( length(Qi),1)];

    % Active inequality constraint Lagrange multipliers
    if ~isempty( info_prev) && dmpc.useActiveConstraints
        options.lambda = [info_prev.lambda(1:n*Np); zeros( length( Qi),1)];
        for j=1:length( Qi_prev)
            idx = find( Qi == Qi_prev(j));
            options.lambda(n*Np+idx) = info_prev.lambda(j);
        end
        options.zl = info_prev.zl;
        options.zu = info_prev.zu;
    end
    
    % Functions
    funcs.objective = optFn;
    funcs.gradient = gradFn;
    funcs.constraints = consFn;
    funcs.jacobian = consJacobian;
    funcs.jacobianstructure = consJacobianStruct;
    funcs.hessian = hessFn;
    funcs.hessianstructure = hessFnStruct;
    
    % Call the solver
    [u_opt,info] = ipopt_auxdata(u0, funcs, options);

end