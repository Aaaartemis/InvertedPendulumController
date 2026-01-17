function u_out = quarc_mpc(x_cur_in)
    tic
    %save the following for optimisation purposes
    persistent controller x_aug delta_u constraints objective parameters_in solutions_out P;
    persistent A_aug B_aug N Q R nx ndeltau;

    if isempty(A_aug)
        % initializing constants 
        ts = 0.002; Rm = 7.5;
        kt = 0.042; km = 0.042;
        mr = 0.095; r = 0.085;
        Jr = (mr*r^2)/3; br = 1e-3;
        mp = 0.024; Lp = 0.129;
        I = Lp/2; Jp = (mp*Lp^2)/3;
        bp = 5e-5; g = 9.81;

        b0 = 1/(mp*I*r);
        b1 = Jp;
        b2 = bp;
        b3 = -mp*g*I;
        b4 = ((km*kt)+(br*Rm))/Rm;
        b5 = Jr+mp*r^2;
        b6 = -kt/Rm;

        den = 1/(1-(b0^2 * b1 * b5));
        t1 = den*b0*b3;
        t2 = den*b0^2 * b1 * b4;
        t3 = den*b0*b2;
        t4 = den*b0^2 * b6 * b1;

        a1 = den*b0^2 * b3 * b5;
        a2 = den*b0*b4;
        a3 = den*b0^2 * b2 * b5;
        a4 = den*b0*b6;

        % A, B matrices from differential equations
        A = [0 0 1 0;
             0 0 0 1;
             0 t1 t2 t3;
             0 a1 a2 a3];

        B = [0;0;t4;a4];

        % discretised with euler method
        Ad = eye(4)+ts*A;
        Bd = ts*B;

        A_aug = [Ad Bd; %augmented state space with u{k-1} for delta u
                 0 0 0 0 1];

        B_aug = [Bd ; 1];

    
        nx = 5; % Number of states (augmented)
        ndeltau = 1;

        % ideal values for Q, R, and N
        Q = [1 0 0 0 0; %slowly center pendulum
             0 1e5 0 0 0; %penalise angular deviation of pendulum most
             0 0 1 0 0;
             0 0 0 1 0;
             0 0 0 0 1e3]; %penalise magnitude of previous input
                        %least possible input signal for all steps

        R = 2; %low penalisation of gradient, we want a fast but smooth response
        N = 50;
    end

    %enforce state as first 5 in input vector
    x_cur_in = (x_cur_in)
    x_cur = x_cur_in(1:5);

    if isempty(controller)
        %2 decision variables, augmented state and change in input
        delta_u = sdpvar(repmat(ndeltau,1,N),repmat(1,1,N));
        x_aug = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

        %DARE for terminal cost
        [K P ev] = dlqr(A_aug,B_aug,Q,R);

        %steady state tracking reduces to stabilisation as we want all
        %states to 0
        %xss is therefore zeros(5,1)

        constraints = [];
        objective = 0;
        for k = 1:N
            % constraints
            objective = objective + x_aug{k}'*Q*x_aug{k} + delta_u{k}'*R*delta_u{k};
            constraints = [constraints, x_aug{k+1} == A_aug*x_aug{k}+B_aug*delta_u{k}];
            constraints = [constraints, -2 <= x_aug{k}(1) <= 2]; % limiting base angle (always)
            %constraints = [constraints, -2 <= x_aug{k}(2) <= 2]; % limitng pendulum angle
            %constraints = [constraints, -3.5 <= x_aug{k}(3) <= 3.5]; %limitng base speed 
            %constraints = [constraints, -2.8 <= x_aug{k}(4) <= 2.8]; % limitng angle speed
            u_k = x_aug{k}(5) + delta_u{k}; % u_k = u_{k-1} + Delta_u_k (always)
            constraints = [constraints, -10 <= u_k <= 10]; % output constraint (always)
        end 
        % defining objective with terminal cost (comment out second part
        % for no P)
        objective = objective + x_aug{N+1}'*P*x_aug{N+1};

        parameters_in = {x_aug{1}};
        solutions_out = {[delta_u{:}], [x_aug{:}]};
        % using "quadprog" solver for optimization problem, osqp can also
        % be used by changing this
        controller = optimizer(constraints, objective,sdpsettings('solver','quadprog'),parameters_in,solutions_out);
    end
 
    inputs = {x_cur};
    [solutions,diagnostics] = controller{inputs};    
    U = solutions{1}; 
    % for infeasibility
    if diagnostics == 1
        disp('The problem is infeasible');
        u_out = 0;
    else
        % if its feasible, update
        u_out(1) = U(1) + x_cur(5); %add delta u to previous u for new input
    end
    u_out(2) = toc;
end
