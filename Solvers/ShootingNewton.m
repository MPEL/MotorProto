classdef ShootingNewton < Solver
    %ShootingNewton.m Time domain periodic steady-state solver
    %
    % ShootingNewton properties:
    %   ShootingTolerance     - Tolerance of the periodic boundary condition
    % 	NewtonTolerance       - Tolerance of iteration occuring at each time point
    % 	GMRESTolerance        - Tolerance of the initial condition correction
    %  	MaxShootingIterations - Maximum number of outer iterations
    %  	MaxNewtonIterations   - Maximum number of iterations at each time point
    %  	MaxGMRESIterations    - Maximum number of iterations for the correction
    %  	RungeKuttaStages      - Number of stages used in the numerical integration
    % 	StoreDecompositions   - Toggle matrix decomposition storage for the GMRES stage
    %
    % ShootingNewton inherits properties and methods Solver.
    %
    % See also MotorProto, Solver
    
%{
properties:
 	%NewtonTolerance - Sets the relative residual tolerance
 	%	NewtonTolerance sets the relative residual tolerance for the Newton-Raphson
    %   iteration that occurs at each time step. The default value is sqrt(eps).
    %
    % See also Static, MaxNewtonIterations
    NewtonTolerance;
    
 	%MaxNewtonIterations - Sets the maximum number of iterations
    %   MaxNewtonIterations sets the maximum number of Newton-Raphson iterations
    %   occuring at each time step. The default value is 100.
    %
    % See also Static, NewtonTolerancerations
    MaxNewtonIterations;
%}

    properties
        ShootingTolerance     = 1e-6;
        NewtonTolerance       = 1e-6;
        GMRESTolerance        = 1e-3;
        MinShootingIterations = 1;
        MaxShootingIterations = 100;
        MinNewtonIterations   = 1;
        MaxNewtonIterations   = 100;
        MaxGMRESIterations    = 100;
        RungeKuttaStages      = 2;
        SymmetricJacobian     = false;
        
        Adaptive               = false;
        SmoothingTolerance     = 1e-2;
        MinSmoothingIterations = 1;
        MaxSmoothingIterations = 2;
        AdaptiveTolerance      = 1e-2;
    end
    
    methods
        %% Constructor
        function this = ShootingNewton(varargin)
            if nargin > 0
                for iArg = 1:2:nargin
                    this.(varargin{iArg}) = varargin{iArg+1};
                end
            end
        end
        
        %% Solve
        function solution = solve(this, model, y0)
            %% Setup Matrices
            getMatrix = DynamicMatrixFactory(copy(model));
            this.Matrices = getMatrix;
            
            Nx = length(getMatrix.f(0));
            
            %% Get RK-Method
            Ns = this.RungeKuttaStages;
            [~,~,c,d,p,q,bu,be,pe] = this.getButcherTable(Ns);
            
            %% Initial Condition
            t        = model.getTimePoints(this.TimePoints);
            Nt       = numel(t) - 1;
         	y        = cell(Ns, Nt+1);
            y_t      = cell(Ns, Nt+1);
            [y{:}]   = deal(zeros(Nx, 1));
            [y_t{:}] = deal(zeros(Nx, 1));
            
            if nargin < 3 || isempty(y0)
                y{end,end}  = zeros(Nx, 1);
            elseif iscell(y0)
                assert(length(y0{end, 1}) == Nx);
            	y{end,1} = y0{end, 1};
            else
                assert(length(y0) == Nx);
                y{end,1} = y0;
            end
            
            %% Begin Simulation
            if this.Verbose
                display(sprintf('\nShooting-Newton %d/%d, Adaptive = %d',Ns,Nt,this.Adaptive));
            end
            
            tic;
            if this.StoreDecompositions
                [y,y_t,t,discErr] = this.solve_stored(y, y_t, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix);
            else
            	[y,y_t,t,discErr] = this.solve_unstored(y, y_t, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix);
            end
            this.SimulationTime = toc;
            this.DiscretizationError = discErr;
            
           	if this.Verbose
                display(sprintf('\nSimulation Time = %0.3g seconds', this.SimulationTime));
            end
            
            %% Post Processing
            Nt  = length(t) - 1;
            x   = cell(1, Nt+1);
            x_t = cell(1, Nt+1);
            for k = 1:Nt
                x{k+1}   = y{end,k+1};
                x_t{k+1} = y_t{end,k+1};
            end
            x{1}   = x{end};
            x_t{1} = x_t{end};
            
            this.Times = t;
            [x, x_t] = getMatrix.doPostProcessing(x, x_t);
            this.Y   = y;
            this.Y_t = y_t;
            this.X   = x;
            this.X_t = x_t;
            solution = Solution(this);
        end
        
        function [y,y_t,t,discErr] = solve_unstored(this, y, y_t, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix)
            %% Minimal Matrix Storage
            Ca = cell(1, Ns);
            
           	%% Start
            first = true;
            if this.Adaptive
                discErr = 1;
                atol = 1;
                last = false;
            else
                last = true;
            end
            
            while first || ~last
                if first% || last
                    shootingTol = this.ShootingTolerance;
                    minShooting = this.MinShootingIterations;
                    maxShooting = this.MaxShootingIterations;
                else
                    shootingTol = this.SmoothingTolerance;
                    minShooting = this.MinSmoothingIterations;
                    maxShooting = this.MaxSmoothingIterations;
                end
                
                ys0 = y{Ns,Nt};%needed for updating y_t{Ns,1} after GMRES
                
                nShooting   = 1;
                shootingRes = 1;
                while (shootingRes > shootingTol && nShooting <= maxShooting) || (minShooting >= nShooting)
                    %% Calculate Residual
                    for k = 2:(Nt+1)
                        h_k = t(k) - t(k-1);
                        for i = 1:Ns
                            t_ik = t(k-1) + c(i) * h_k;
                            h_ik = h_k / d(i,i);

                            y{i,k} = y{Ns,k-1};

                            [~, rpq] = getMatrix.G(t(k-1), y{Ns,k-1}, h_ik);
                            Kpq = getMatrix.K(t(k-1), h_ik);
                            Cpq = getMatrix.C(t(k-1), h_k / p(i), h_ik);
                            rpq = rpq + Kpq * y{Ns,k-1};
                            rpq = rpq - getMatrix.f(t(k-1), h_ik);
                            rpq = q(i) * rpq - Cpq * y{Ns,k-1};

                            for j = 1:i
                                Ca{j} = getMatrix.C(t_ik, h_k / d(i,j), h_ik);
                            end

                            f_ik = getMatrix.f(t_ik, h_ik);
                            K_ik = getMatrix.K(t_ik, h_ik);

                            %% Newton Iteration
                            nNewton   = 1;
                            newtonRes = 1;
                            while newtonRes >= this.NewtonTolerance && nNewton <= this.MaxNewtonIterations
                                nNewton = nNewton + 1;
                                [G_ik, g_ik] = getMatrix.G(t_ik, y{i,k}, h_ik);

                                J_ik = K_ik + Ca{i} + G_ik;
                                r_ik = K_ik * y{i,k} + g_ik - f_ik + rpq;
                                for j = 1:i
                                    r_ik = r_ik + Ca{j} * y{j,k};
                                end

                                newtonRes = norm(r_ik) / norm(g_ik - f_ik);

                                r_ik   = J_ik \ r_ik;
                                y{i,k} = y{i,k} - r_ik;
                            end

                            %% Calculate Stage Derivative
                            y_t{i,k} = (-p(i)/h_k) * y{Ns,k-1} - q(i)*y_t{Ns,k-1};
                            for j = 1:i
                                y_t{i,k} = y_t{i,k} + (d(i,j)/h_k)*y{j,k};
                            end
                        end
                    end
                    r    = cell(Ns+1,1);
                    r{1} = ys0 - y{Ns,Nt};
                    for i = 1:Ns
                        r{i+1} = y{i,1} - y{i,Nt+1};
                    end
                    r = cell2mat(r);
                    
                    shootingRes = norm(r) / norm(cell2mat(y(:,Nt+1)));

                    %% Calculate Error Estimates
                    [ec, discErr] = this.rkErrorCoefficients(t, y(:,2:end), y_t(:,2:end), be, pe, getMatrix);
                    if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nShooting == 1)
%                         shootingTol = this.ShootingTolerance;
%                         minShooting = this.MinShootingIterations;
%                         maxShooting = this.MaxShootingIterations;
                        last = true;
                    end
                    
                    %% Report
                    if this.Verbose
                        display(sprintf('\nIteration %d, Residual = %0.3g, Discretization Error = %0.3g', nShooting, shootingRes, discErr));
                    end

                    %% GMRES Phase
                    if (shootingRes > shootingTol) && (this.MaxGMRESIterations > 0)
                        A = @ShootingNewton.MVPUnstored;
                        if this.Verbose
                            dy = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix);
                        else
                            [dy,~,~,~] = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix);
                        end
                        
                        dy = mat2cell(dy,Nx*ones(Ns+1,1),1);
                        ys0 = ys0 - dy{1};
                        for i = 1:Ns
                            y{i,1} = y{i,1} - dy{i+1};
                        end
                        h_k = t(Nt+1) - t(Nt);
                        y_t{Ns,1} = (-p(Ns)/h_k) * ys0;
                        for j = 1:Ns
                            y_t{Ns,1} = y_t{Ns,1} + (d(Ns,j)/h_k)*y{j,1};
                        end
                    elseif (this.MaxGMRESIterations == 0) || (minShooting >= nShooting)
                        y(:,1)   = y(:,Nt+1);
                        ys0      = y{Ns,Nt};
                        y_t(:,1) = y_t(:,Nt+1);
                    end

                    nShooting = nShooting + 1;
                end
                
                %% Refine Grid
                if this.Adaptive && ~last
                    [s, atol] = this.rkRefine(t, atol, this.AdaptiveTolerance, ec, pe, 2, first);
                  	[y, y_t, t] = this.rkInterpolate(t, y(:,2:end), y_t(:,2:end), c, bu, s);
                 	y   = cat(2,y(:,end),y);
                    y_t = cat(2,y_t(:,end),y_t);
                    Nt  = length(t) - 1;
                    
                    if this.Verbose
                        display(sprintf('\nRefining grid to %d time-steps', Nt));
                    end
                end
                first = first && ~first;
            end
        end

        function [y,y_t,t,discErr] = solve_stored(this, y, y_t, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix)
            first = true;
            if this.Adaptive
                atol = 1;
                last = false;
            else
                last = true;
            end
            
            %% Store Everything
            K   = cell(Ns, Nt+1);
            f   = cell(Ns, Nt+1);
            Ca  = cell(Ns, Ns, Nt+1);
            Jpq = cell(Ns, Nt+1);
            if this.SymmetricJacobian
                L = cell(Ns, Nt+1);
                D = cell(Ns, Nt+1);
                S = cell(Ns, Nt+1);
                P = cell(Ns, Nt+1);
            else
                L = cell(Ns, Nt+1);
                U = cell(Ns, Nt+1);
                P = cell(Ns, Nt+1);
                Q = cell(Ns, Nt+1);
                R = cell(Ns, Nt+1);
            end
            
            while first || ~last
                %% Preallocate Matrices
                for k = 2:(Nt+1)
                    h_k  = t(k) - t(k-1);
                    for i = 1:Ns
                        t_ik = t(k-1) + c(i) * h_k;
                        h_ik = h_k / d(i,i);
                        for j = 1:i
                            Ca{i,j,k} = getMatrix.C(t_ik, h_k / d(i,j), h_ik);
                        end
                        K{i,k}  = getMatrix.K(t_ik, h_ik);
                        f{i,k}  = getMatrix.f(t_ik, h_ik);
                    end
                end
                
                if first% || last
                    shootingTol = this.ShootingTolerance;
                    minShooting = this.MinShootingIterations;
                    maxShooting = this.MaxShootingIterations;
                else
                    shootingTol = this.SmoothingTolerance;
                    minShooting = this.MinSmoothingIterations;
                    maxShooting = this.MaxSmoothingIterations;
                end
                
                ys0 = y{Ns,Nt};%needed for updating y_t{Ns,1} after GMRES
                
                nShooting   = 1;
                shootingRes = 1;
                while (shootingRes > shootingTol && nShooting <= maxShooting) || (minShooting >= nShooting) 
                    %% Calculate Residual
                    for k = 2:(Nt+1)
                        h_k = t(k) - t(k-1);
                        for i = 1:Ns
                            t_ik = t(k-1) + c(i) * h_k;
                            h_ik = h_k / d(i,i);

                            y{i, k} = y{Ns, k-1};

                            [Gpq, rpq] = getMatrix.G(t(k-1), y{Ns,k-1}, h_ik);
                            Kpq        = getMatrix.K(t(k-1), h_ik);
                            Cpq        = getMatrix.C(t(k-1), h_k / p(i), h_ik);
                            Jpq{i,k}   = Cpq - q(i) * (Gpq + Kpq);
                            rpq        = rpq + Kpq * y{Ns,k-1};
                            rpq        = rpq - getMatrix.f(t(k-1), h_ik);
                            rpq        = q(i) * rpq - Cpq * y{Ns,k-1};

                            %% Newton Iteration
                            nNewton   = 1;
                            newtonRes = 1;
                            while newtonRes > this.NewtonTolerance && nNewton <= this.MaxNewtonIterations
                                nNewton = nNewton + 1;

                                [G_ik, g_ik] = getMatrix.G(t_ik, y{i,k}, h_ik);

                                r_ik = K{i,k} * y{i,k} + g_ik - f{i,k} + rpq;
                                for j = 1:i
                                    r_ik = r_ik + Ca{i,j,k} * y{j,k};
                                end

                                newtonRes = norm(r_ik) / norm(g_ik - f{i,k});

                                J_ik = K{i,k} + Ca{i,i,k} + G_ik;
                                if this.SymmetricJacobian
                                    [L_ik, D_ik, P_ik, S_ik] = ldl(J_ik);
                                    r_ik = S_ik  * r_ik;
                                    r_ik = P_ik' * r_ik;
                                    r_ik = L_ik  \ r_ik;
                                    r_ik = D_ik  \ r_ik;
                                    r_ik = L_ik' \ r_ik;
                                    r_ik = P_ik  * r_ik;
                                    r_ik = S_ik  * r_ik;
                                else
                                    [L_ik, U_ik, P_ik, Q_ik, R_ik] = lu(J_ik);
                                    r_ik = R_ik \ r_ik;
                                    r_ik = P_ik * r_ik;
                                    r_ik = L_ik \ r_ik;
                                    r_ik = U_ik \ r_ik;
                                    r_ik = Q_ik * r_ik;
                                end
                                y{i,k} = y{i,k} - r_ik;
                            end

                            %% Calcualte Stage Derivative
                            y_t{i,k} = (-p(i)/h_k) * y{Ns,k-1} - q(i)*y_t{Ns,k-1};
                            for j = 1:i
                                y_t{i,k} = y_t{i,k} + (d(i,j)/h_k)*y{j,k};
                            end

                            if this.SymmetricJacobian
                                L{i,k} = L_ik;
                                D{i,k} = D_ik;
                                P{i,k} = P_ik;
                                S{i,k} = S_ik;
                            else
                                L{i,k} = L_ik;
                                U{i,k} = U_ik;
                                P{i,k} = P_ik;
                                Q{i,k} = Q_ik;
                                R{i,k} = R_ik;
                            end
                        end
                    end
                    r    = cell(Ns+1,1);
                    r{1} = ys0 - y{Ns,Nt};
                    for i = 1:Ns
                        r{i+1} = y{i,1} - y{i,Nt+1};
                    end
                    r = cell2mat(r);
                    
                    shootingRes = norm(r) / norm(cell2mat(y(:,Nt+1)));

                    %% Calculate Error Estimates
                    [ec, discErr] = this.rkErrorCoefficients(t, y(:,2:end), y_t(:,2:end), be, pe, getMatrix);
                    if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nShooting == 1)
%                         shootingTol = this.ShootingTolerance;
%                         minShooting = this.MinShootingIterations;
%                         maxShooting = this.MaxShootingIterations;
                        last = true;
                    end
                    
                    %% Report
                    if this.Verbose
                        display(sprintf('\nIteration %d, Residual = %0.3g, Discretization Error = %0.3g', nShooting, shootingRes, discErr));
                    end
                    
                    %% GMRES Phase
                    if (shootingRes > shootingTol) && (this.MaxGMRESIterations > 0)
                        if this.SymmetricJacobian
                            A = @ShootingNewton.MVPStoredLDL;
                            if this.Verbose
                                dy = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, Ca, Jpq, L, D, P, S, Ns, Nt, Nx);
                            else
                                [dy,~,~,~] = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, K, Ca, Jpq, L, D, P, S, Ns, Nt, Nx);
                            end
                        else
                            A = @ShootingNewton.MVPStoredLU;
                            if this.Verbose
                                dy = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, Ca, Jpq, L, U, P, Q, R, Ns, Nt, Nx);
                            else
                                [dy,~,~,~] = gmres(A, r, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r, Ca, Jpq, L, U, P, Q, R, Ns, Nt, Nx);
                            end
                        end
                        dy = mat2cell(dy,Nx*ones(Ns+1,1),1);
                        ys0 = ys0 - dy{1};
                        for i = 1:Ns
                            y{i,1} = y{i,1} - dy{i+1};
                        end
                        h_k = t(Nt+1) - t(Nt);
                        y_t{Ns,1} = (-p(Ns)/h_k) * ys0;
                        for j = 1:Ns
                            y_t{Ns,1} = y_t{Ns,1} + (d(Ns,j)/h_k)*y{j,1};
                        end
                    elseif (this.MaxGMRESIterations == 0) || (minShooting >= nShooting)
                        y(:,1)   = y(:,Nt+1);
                        ys0      = y{Ns,Nt};
                        y_t(:,1) = y_t(:,Nt+1);
                    end
                    
                    nShooting = nShooting + 1;
                end
                
                %% Refine Grid
                if this.Adaptive && ~last
                    [s, atol] = this.rkRefine(t, atol, this.AdaptiveTolerance, ec, pe, 2, first);
                  	[y, y_t, t] = this.rkInterpolate(t, y(:,2:end), y_t(:,2:end), c, bu, s);
                    y  = cat(2,y(:,end),y);
                    y_t = cat(2,y_t(:,end),y_t);
                    Nt = length(t) - 1;
                    
                    if this.Verbose
                        display(sprintf('\nRefining grid to %d time-steps', Nt));
                    end
                end
                
                first = first && ~first;
            end
        end
    end
    
    methods (Static)
        %% Stored Matrices
        function mvp = MVPStoredLDL(z_s0, Ca, Jpq, L, D, P, S, Ns, Nt, Nx)
            z_s0 = mat2cell(z_s0,Nx*ones(Ns+1,1),1);
            z    = cell(Ns,1);
            z{Ns} = z_s0{Ns+1};
            
            for k = 2:(Nt+1)
                z_sk = z{Ns};  
                for i = 1:Ns         
                    rhs = Jpq{i,k} * z_sk;
                    for j = 1:(i - 1);
                        rhs = rhs - Ca{i,j,k} * z{j};
                    end
                    z{i} = S{i,k} * (P{i,k} * (L{i,k}' \ (D{i,k} \ (L{i,k} \ (P{i,k}' * (S{i,k} * rhs))))));
                end
            end
            
            mvp = cell(Ns+1,1);
            mvp{1} = z_s0{1} - z_sk;
            for i = 1:Ns
                mvp{i+1} = z_s0{i+1} - z{i};
            end
            mvp = cell2mat(mvp);
        end
        
        function mvp = MVPStoredLU(z_s0, Ca, Jpq, L, U, P, Q, R, Ns, Nt, Nx)
            z_s0 = mat2cell(z_s0,Nx*ones(Ns+1,1),1);
            z    = cell(Ns,1);
            z{Ns} = z_s0{Ns+1};
            
            for k = 2:(Nt+1)
                z_sk = z{Ns};
                for i = 1:Ns         
                    r_ik = Jpq{i,k} * z_sk;
                    for j = 1:(i - 1);
                        r_ik = r_ik - Ca{i,j,k} * z{j};
                    end
                    z{i} = Q{i,k} * (U{i,k} \ (L{i,k} \ (P{i,k} * (R{i,k} \ r_ik))));
                end
            end
            
            mvp = cell(Ns+1,1);
            mvp{1} = z_s0{1} - z_sk;
            for i = 1:Ns
                mvp{i+1} = z_s0{i+1} - z{i};
            end
            mvp = cell2mat(mvp);
        end
        
        %% Unstored Matrices
        function mvp = MVPUnstored(z_s0, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix)
            z_s0 = mat2cell(z_s0,Nx*ones(Ns+1,1),1);
            z    = cell(Ns,1);
            z{Ns} = z_s0{Ns+1};
            
            for k = 2:(Nt+1)
                z_sk = z{Ns};  
                h_k  = t(k) - t(k-1);
                for i = 1:Ns
                    t_ik = t(k-1) + c(i) * h_k;
                    h_ik = h_k / d(i,i);
                    
                    Jpq = getMatrix.G(t(k-1), y{Ns,k-1}, h_ik);
                    Jpq = Jpq + getMatrix.K(t(k-1), h_ik);
                    Jpq = getMatrix.C(t(k-1), h_k / p(i), h_ik) - q(i)*Jpq;
                    r_ik = Jpq * z_sk;
                    for j = 1:(i-1)
                        C    = getMatrix.C(t_ik, h_k / d(i,j), h_ik);
                        r_ik = r_ik - C * z{j};
                    end

                  	J_ik = getMatrix.K(t_ik, h_ik);
                    J_ik = J_ik + getMatrix.G(t_ik, y{i,k}, h_ik);
                    J_ik = J_ik + getMatrix.C(t_ik, h_ik, h_ik);
                    
                    z{i} = J_ik \ r_ik;
                end               
            end
            
            mvp = cell(Ns+1,1);
            mvp{1} = z_s0{1} - z_sk;
            for i = 1:Ns
                mvp{i+1} = z_s0{i+1} - z{i};
            end
            mvp = cell2mat(mvp);
        end
      
        %% Configure
        function solverOut = configureSolver(varargin)
            if nargin > 0
                solverOut = ShootingNewton(varargin{:});
            end
        end
    end
end