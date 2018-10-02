classdef TPFEM < Solver
    %TPFEM.m Time-Periodic FEM steady-state solver
    %
    % TPFEM properties:
    % 	NewtonTolerance       - Tolerance of iteration occuring at each time point
    % 	GMRESTolerance        - Tolerance of the initial condition correction
    %  	MaxNewtonIterations   - Maximum number of iterations at each time point
    %  	MaxGMRESIterations    - Maximum number of iterations for the correction
    %  	RungeKuttaStages      - Number of stages used in the numerical integration
    % 	StoreDecompositions   - Toggle matrix decomposition storage for the GMRES stage
    %
    % ShootingNewton inherits properties and methods Solver.
    %
    % See also MotorProto, Solver,
    
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
        NewtonTolerance     = 1e-6;
        GMRESTolerance      = 1e-3;
        MaxNewtonIterations = 100;
        MinNewtonIterations = 1;
        MaxGMRESIterations  = 100;
        RungeKuttaStages    = 2;
        SymmetricJacobian   = true;
        
        Adaptive               = false;
        SmoothingTolerance     = 1e-2;
        MinSmoothingIterations = 1;
        MaxSmoothingIterations = 2;
        AdaptiveTolerance      = 1e-3;
    end
    
    methods
        %% Constructor
        function this = TPFEM(varargin)
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
            
            %% Get Algorithm Parameters
            Ns = this.RungeKuttaStages;
            [~,~,c,d,p,q,bu,be,pe] = this.getButcherTable(Ns);
            
            %% Initialize
            t  = model.getTimePoints(this.TimePoints);
            Nt = numel(t) - 1;

            r = cell(Ns, Nt);
            y_t = cell(Ns, Nt);
            [r{:}] = deal(zeros(Nx, 1));
            [y_t{:}] = deal(zeros(Nx, 1));
            if nargin == 3
                y = y0(:,1:Nt);
            else
                y = cell(Ns, Nt);
                [y{:}] = deal(zeros(Nx, 1));
            end
            
            %% Begin Simulation Timing
            if this.Verbose
                display(sprintf('TPFEM %d/%d\n',Ns,Nt));
            end
            
            tic;
            if this.StoreDecompositions
                [y,y_t,t,discErr] = this.solve_stored(y, y_t, r, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix);
            else
            	[y,y_t,t,discErr] = this.solve_unstored(y, y_t, r, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix);
            end
            this.SimulationTime = toc;
            this.DiscretizationError = discErr;
            
           	if this.Verbose
                display(sprintf('Simulation Time = %0.3g seconds\n', this.SimulationTime));
            end
            
            %% Post Processing
            Nt  = numel(t) - 1;
            x   = cell(1, Nt+1);
            x_t = cell(1, Nt+1);
            for k = 1:Nt
                x{k+1}   = y{end,k};
                x_t{k+1} = y_t{end,k};
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
        
        function [y, y_t, t, discErr] = solve_unstored(this, y, y_t, r, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix)
           	first = true;
            discErr = 1;
            if this.Adaptive
                atol = 1;
                last = false;
            else
                last = true;
            end
            
            while first || ~last
                if first% || last
                    newtonTol = this.NewtonTolerance;
                    minNewton = this.MinNewtonIterations;
                    maxNewton = this.MaxNewtonIterations;
                else
                    newtonTol = this.SmoothingTolerance;
                    minNewton = this.MinSmoothingIterations;
                    maxNewton = this.MaxSmoothingIterations;
                end
                
                %% Newton Iteration
                nNewton = 1;
                newtonRes = 1;
                while (newtonRes > newtonTol && nNewton <= maxNewton) || (minNewton >= nNewton)
                    %% Calculate Residual
                    normf = 0;
                    normr = 0;
                    for k = 1:Nt
                        km1 = mod(k-2, Nt) + 1;
                        h_k = t(k+1) - t(k);
                        for i = 1:Ns
                            t_ik = t(k) + c(i) * h_k;
                            h_ik = h_k / d(i,i);
                            
                            [~, rpq] = getMatrix.G(t(k), y{Ns,km1}, h_ik);
                            Kpq = getMatrix.K(t(k), h_ik);
                            Cpq = getMatrix.C(t(k), h_k / p(i), h_ik);
                            
                            r{i,k} = rpq + Kpq * y{Ns,km1};
                            r{i,k} = r{i,k} - getMatrix.f(t(k), h_ik);
                            r{i,k} = q(i) * r{i,k} - Cpq * y{Ns,km1};
                            
                            K_ik = getMatrix.K(t_ik, h_ik);
                            r{i,k} = r{i,k} + K_ik * y{i,k};
                            
                            f_ik = getMatrix.f(t_ik, h_ik);
                            r{i,k} = r{i,k} - f_ik;
                            
                            [~, g_ik] = getMatrix.G(t_ik, y{i,k}, h_ik);
                            r{i,k} = r{i,k} + g_ik;
                            
                            for j = 1:i
                                C      = getMatrix.C(t_ik, h_k / d(i,j), h_ik);
                                r{i,k} = r{i,k} + C * y{j,k};
                            end
                            
                            normf = normf + norm(f_ik)^2;
                            normr = normr + norm(r{i,k})^2;
                        end
                    end
                    newtonRes = sqrt(normr / normf);
                    
                    %% Report
                    if this.Verbose
                        display(sprintf('\nIteration %d, Residual = %0.3g, Discretization Error = %0.3g', nNewton, newtonRes, discErr));
                    end
                    
                    %% GMRES Phase
                    if (newtonRes > newtonTol) || (minNewton >= nNewton)
                        dy = reshape(r, [], 1);
                        dy = cell2mat(dy);
                        A = @TPFEM.MVPUnstored;
                        
                        if this.Verbose
                            dy = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix);
                        else
                            [dy,~,~,~] = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix);
                        end
                        dy = TPFEM.PCUnstored(dy, y, t, c, d, p, q, Ns, Nt, Nx, getMatrix);
                        dy = mat2cell(dy, Nx * ones(1, Nt*Ns));
                        dy = reshape(dy, Ns, Nt);
                        
                        for i = 1:Nt
                            for j = 1:Ns
                                y{j,i} = y{j,i} - dy{j,i};
                            end
                        end
                    end
                    
                    %% Calculate Stage Derivatives
                    for i = Ns:-1:1
                        for k = 1:Nt
                            km1 = mod(k-2, Nt) + 1;
                            h_k = t(k+1) - t(k);
                            y_t{i,k} = (-p(i)/h_k) * y{Ns,km1} - q(i)*y_t{Ns,km1};
                            for j = 1:i
                                y_t{i,k} = y_t{i,k} + (d(i,j)/h_k)*y{j,k};
                            end
                        end
                    end
                    
                  	%% Calculate Error Estimates
                    [ec, discErr] = this.rkErrorCoefficients(t, y, y_t, be, pe, getMatrix);
                    if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nNewton == 1)
%                         newtonTol = this.NewtonTolerance;
%                         minNewton = this.MinNewtonIterations;
%                         maxNewton = this.MaxNewtonIterations;
                        last = true;
                    end
                    
                    nNewton = nNewton + 1;
                end
                
                %% Refine Grid
                if this.Adaptive && ~last
                    [s, atol] = this.rkRefine(t, atol, this.AdaptiveTolerance, ec, pe, 2, first);
                  	[y, y_t, t] = this.rkInterpolate(t, y, y_t, c, bu, s);
                    Nt = length(t) - 1;
                    
                    if this.Verbose
                        display(sprintf('\nRefining grid to %d time-steps', Nt));
                    end
                end
                first = first && ~first;
            end
        end
   
        function [y, y_t, t, discErr] = solve_stored(this, y, y_t, r, t, c, d, p, q, bu, be, pe, Nt, Ns, Nx, getMatrix)
           	first = true;
            discErr = 1;
            if this.Adaptive
                atol = 1;
                last = false;
            else
                last = true;
            end
            
            K  = cell(Ns, Nt);
            f  = cell(Ns, Nt);
            Cd = cell(Ns, Ns, Nt);
            Jpq = cell(Ns, Nt);
            
            if this.SymmetricJacobian
                L = cell(Ns, Nt);
                D = cell(Ns, Nt);
                S = cell(Ns, Nt);
                P = cell(Ns, Nt);
            else
                L = cell(Ns, Nt);
                U = cell(Ns, Nt);
                P = cell(Ns, Nt);
                Q = cell(Ns, Nt);
                R = cell(Ns, Nt);
            end
            
            while first || ~last
                if first% || last
                    newtonTol = this.NewtonTolerance;
                    minNewton = this.MinNewtonIterations;
                    maxNewton = this.MaxNewtonIterations;
                else
                    newtonTol = this.SmoothingTolerance;
                    minNewton = this.MinSmoothingIterations;
                    maxNewton = this.MaxSmoothingIterations;
                end
                
                for k = 1:Nt
                    h_k = t(k+1) - t(k);
                    for i = 1:Ns
                        t_ik = t(k) + c(i) * h_k;
                        h_ik = h_k / d(i,i);

                        K{i,k} = getMatrix.K(t_ik, h_ik);
                        f{i,k} = getMatrix.f(t_ik, h_ik);

                        for j = 1:i
                            Cd{i,j,k} = getMatrix.C(t_ik, h_k / d(i,j), h_ik);
                        end
                    end
                end
                    
                %% Newton Iterations
                nNewton   = 1;
                newtonRes = 1;
                while (newtonRes > newtonTol && nNewton <= maxNewton) || (minNewton >= nNewton)
                    %% Calculate Residual
                    normf = 0;
                    normr = 0;
                    for k = 1:Nt
                        km1 = mod(k-2, Nt) + 1;
                        h_k = t(k+1) - t(k);
                        for i = 1:Ns
                            t_ik = t(k) + c(i) * h_k;
                            h_ik = h_k / d(i,i);

                            [Gpq, rpq] = getMatrix.G(t(k), y{Ns,km1}, h_ik);
                            Kpq        = getMatrix.K(t(k), h_ik);
                            Cpq        = getMatrix.C(t(k), h_k / p(i), h_ik);
                            Jpq{i,k}   = Cpq - q(i) * (Gpq + Kpq);
                            rpq        = rpq + Kpq * y{Ns,km1};
                            rpq        = rpq - getMatrix.f(t(k), h_ik);
                            rpq        = q(i) * rpq - Cpq * y{Ns,km1};

                            [G_ik, g_ik] = getMatrix.G(t_ik, y{i,k}, h_ik);

                            r{i,k} = K{i,k} * y{i,k} + g_ik - f{i,k} + rpq;
                            for j = 1:i
                                r{i,k} = r{i,k} + Cd{i,j,k} * y{j,k};
                            end

                            J_ik = K{i,k} + Cd{i,i,k} + G_ik;
                            if this.SymmetricJacobian
                                [L{i,k}, D{i,k}, P{i,k}, S{i,k}] = ldl(J_ik);
                            else
                                [L{i,k}, U{i,k}, P{i,k}, Q{i,k}, R{i,k}] = lu(J_ik);
                            end

                            normf = normf + norm(f{i,k})^2;
                            normr = normr + norm(r{i,k})^2;
                        end
                    end
                    newtonRes = sqrt(normr / normf);
                    
                    %% Report
                    if this.Verbose
                        display(sprintf('\nIteration %d, Residual = %0.3g, Discretization Error = %0.3g', nNewton, newtonRes, discErr));
                    end
                    
                    %% GMRES Phase
                    if (newtonRes > newtonTol) || (minNewton >= nNewton)
                        dy = reshape(r,[],1);
                        dy = cell2mat(dy);

                        if this.SymmetricJacobian
                            A = @TPFEM.MVPStoredLDL;
                            if this.Verbose
                                dy = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, Cd, Jpq, L, D, P, S, Ns, Nt, Nx);
                            else
                                [dy,~,~,~] = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, Cd, Jpq, L, D, P, S, Ns, Nt, Nx);
                            end
                            dy = TPFEM.PCStoredLDL(dy, Cd, Jpq, L, D, P, S, Ns, Nt, Nx);
                        else
                            A = @TPFEM.MVPStoredLU;
                            if this.Verbose
                                dy = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, Cd, Jpq, L, U, P, Q, R, Ns, Nt, Nx);
                            else
                                [dy,~,~,~] = gmres(A, dy, this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], dy, Cd, Jpq, L, U, P, Q, R, Ns, Nt, Nx);
                            end
                            dy = TPFEM.PCStoredLU(dy, Cd, Jpq, L, D, P, S, Ns, Nt, Nx);
                        end
                        dy = mat2cell(dy,Nx*ones(1,Nt*Ns));
                        dy = reshape(dy,Ns,Nt);

                        for i = 1:Nt
                            for j = 1:Ns
                                y{j,i} = y{j,i} - dy{j,i};
                            end
                        end
                    end

                    %% Calculate Stage Derivatives
                    for i = Ns:-1:1
                        for k = 1:Nt
                            km1 = mod(k-2, Nt) + 1;
                            h_k = t(k+1) - t(k);
                            y_t{i,k} = (-p(i)/h_k) * y{Ns,km1} - q(i)*y_t{Ns,km1};
                            for j = 1:i
                                y_t{i,k} = y_t{i,k} + (d(i,j)/h_k)*y{j,k};
                            end
                        end
                    end
                    
                  	%% Calculate Error Estimates
                    [ec, discErr] = this.rkErrorCoefficients(t, y, y_t, be, pe, getMatrix);
                    if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nNewton == 1)
%                        	newtonTol = this.NewtonTolerance;
%                         minNewton = this.MinNewtonIterations;
%                         maxNewton = this.MaxNewtonIterations;
                        last = true;
                    end
                    
                    nNewton = nNewton + 1;
                end
                
                %% Refine Grid
                if this.Adaptive && ~last
                    [s, atol] = this.rkRefine(t, atol, this.AdaptiveTolerance, ec, pe, 2, first);
                  	[y, y_t, t] = this.rkInterpolate(t, y, y_t, c, bu, s);
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
        %% Unstored Matricess
     	function y = MVPUnstored(x, z, t, c, d, p, q, Ns, Nt, Nx, getMatrix)
            x = mat2cell(x, Nx * ones(1, Ns * Nt));
            x = reshape(x, Ns, Nt);
            y = x;
            
            %% k = 1 Iteration
            k  = 1;
            h_k = t(k+1) - t(k);
            for i = 1:Ns
                t_ik = t(k) + c(i) * h_k;
                h_ik = h_k / d(i,i);
                
                J_ik = getMatrix.K(t_ik, h_ik);
                J_ik = J_ik + getMatrix.C(t_ik, h_ik, h_ik);
                J_ik = J_ik + getMatrix.G(t_ik, z{i,k}, h_ik);
                
                x{i,k} = J_ik \ x{i,k};
                
                for j = (i+1):Ns
                    C      = getMatrix.C(t_ik, h_k / d(j,i), h_k / d(j,j));
                    x{j,k} = x{j,k} - C * x{i,k};
                end
            end
            
            %% k > 1 Iteration
            for k = 2:Nt
                h_k = t(k+1) - t(k);
                for i = 1:Ns
                    t_ik = t(k) + c(i) * h_k;
                    h_ik = h_k / d(i,i);
                
                   	Jpq = getMatrix.G(t(k), z{Ns,k-1}, h_ik);
                    Jpq = Jpq + getMatrix.K(t(k), h_ik);
                    Jpq = getMatrix.C(t(k), h_k / p(i), h_ik) - q(i)*Jpq;
                    x{i,k} = x{i,k} + Jpq * x{Ns, k-1};
                    
                    J_ik = getMatrix.K(t_ik, h_ik);
                    J_ik = J_ik + getMatrix.C(t_ik, h_ik, h_ik);
                    J_ik = J_ik + getMatrix.G(t_ik, z{i,k}, h_ik);
                    
                    x{i,k} = J_ik \ x{i,k};
                    
                    for j = (i+1):Ns
                        C      = getMatrix.C(t_ik, h_k / d(j,i), h_k / d(j,j));
                        x{j,k} = x{j,k} - C * x{i,k}; 
                  	end
                end
            end
            
            %% Matrix contribution from upper-right hand corner block
            k  = 1;
            h_k = t(k+1) - t(k);
            for i = 1:Ns
                h_ik = h_k / d(i,i);
                
                Jpq = getMatrix.G(t(k), z{Ns,Nt}, h_ik);
                Jpq = Jpq + getMatrix.K(t(k), h_ik);
                Jpq = getMatrix.C(t(k), h_k / p(i), h_ik) - q(i)*Jpq;
                y{i,1} = y{i,1} - Jpq * x{Ns, Nt};
            end
            
            y = reshape(y, Ns * Nt, 1);
            y = cell2mat(y);
        end
        
        function x = PCUnstored(x, z, t, c, d, p, q, Ns, Nt, Nx, getMatrix)
            x = mat2cell(x, Nx * ones(1, Ns * Nt));
            x = reshape(x, Ns, Nt);
            
            %% k = 1 Iteration
            k  = 1;
            h_k = t(k+1) - t(k);
            for i = 1:Ns
                t_ik = t(k) + c(i) * h_k;
                h_ik = h_k / d(i,i);
                
                J_ik = getMatrix.K(t_ik, h_ik);
                J_ik = J_ik + getMatrix.C(t_ik, h_ik, h_ik);
                J_ik = J_ik + getMatrix.G(t_ik, z{i,k}, h_ik);
                
                x{i,k} = J_ik \ x{i,k};
                
                for j = (i+1):Ns
                    C      = getMatrix.C(t_ik, h_k / d(j,i), h_k / d(j,j));
                    x{j,k} = x{j,k} - C * x{i,k};
                end
            end
            
            %% k > 1 Iteration
            for k = 2:Nt
                h_k = t(k+1) - t(k);
                for i = 1:Ns
                    t_ik = t(k) + c(i) * h_k;
                    h_ik = h_k / d(i,i);
                
                    Jpq = getMatrix.G(t(k), z{Ns,k-1}, h_ik);
                    Jpq = Jpq + getMatrix.K(t(k), h_ik);
                    Jpq = getMatrix.C(t(k), h_k / p(i), h_ik) - q(i)*Jpq;
                    x{i,k} = x{i,k} + Jpq * x{Ns, k-1};
                    
                    J_ik = getMatrix.K(t_ik, h_ik);
                    J_ik = J_ik + getMatrix.C(t_ik, h_ik, h_ik);
                    J_ik = J_ik + getMatrix.G(t_ik, z{i,k}, h_ik);
                    
                    x{i,k} = J_ik \ x{i,k};
                    
                    for j = (i+1):Ns
                        C      = getMatrix.C(t_ik, h_k / d(j,i), h_k / d(j,j));
                        x{j,k} = x{j,k} - C * x{i,k}; 
                  	end
                end
            end
            
            x = reshape(x, Ns * Nt, 1);
            x = cell2mat(x);
        end
        
        %% Stored Matrices
        function y = MVPStoredLDL(x, Cd, Jpq, L, D, P, S, Ns, Nt, Nx)
            x = mat2cell(x, Nx * ones(1, Ns * Nt));
            x = reshape(x, Ns, Nt);
            y = x;
            
            %% k = 1 Iteration Setup
            k = 1;
            for j = 1:Ns
                x{j,k} = S{j,k} * (P{j,k} * (L{j,k}' \ (D{j,k} \ (L{j,k} \ (P{j,k}' * (S{j,k} * x{j,k}))))));
                for i = (j+1):Ns
                    x{i,k} = x{i,k} - Cd{i,j,k} * x{j,k};
                end
            end
            
            %% k > 1 Iterations
            for k = 2:Nt
                for j = 1:Ns
                    x{j,k} = x{j,k} + Jpq{j,k} * x{Ns,k-1};
                    x{j,k} = S{j,k} * (P{j,k} * (L{j,k}' \ (D{j,k} \ (L{j,k} \ (P{j,k}' * (S{j,k} * x{j,k}))))));
                    for i = (j+1):Ns
                        x{i,k} = x{i,k} - Cd{i,j,k} * x{j,k}; 
                  	end
                end
            end
            
            %% Matrix contribution from upper-right hand corner block
            for j = 1:Ns
                y{j,1} = y{j,1} - Jpq{j,1} * x{Ns,Nt};
            end
            
            y = reshape(y, Ns * Nt, 1);
            y = cell2mat(y);
        end
        
        function x = PCStoredLDL(x, Cd, Jpq, L, D, P, S, Ns, Nt, Nx)
            x = mat2cell(x, Nx * ones(1, Ns * Nt));
            x = reshape(x, Ns, Nt);
            
            %% k = 1 Iteration Setup
            k = 1;
            for j = 1:Ns
                x{j,k} = S{j,k} * (P{j,k} * (L{j,k}' \ (D{j,k} \ (L{j,k} \ (P{j,k}' * (S{j,k} * x{j,k}))))));
                for i = (j+1):Ns
                    x{i,k} = x{i,k} - Cd{i,j,k} * x{j,k};
                end
            end
            
            %% k > 1 Iterations
            for k = 2:Nt
                for j = 1:Ns
                    x{j,k} = x{j,k} + Jpq{j,k} * x{Ns,k-1};
                    x{j,k} = S{j,k} * (P{j,k} * (L{j,k}' \ (D{j,k} \ (L{j,k} \ (P{j,k}' * (S{j,k} * x{j,k}))))));
                    for i = (j+1):Ns
                        x{i,k} = x{i,k} - Cd{i,j,k} * x{j,k}; 
                  	end
                end
            end
            
            x = reshape(x, Ns * Nt, 1);
            x = cell2mat(x);
        end
        
        function y = MVPStoredLU(x, Ca, Cg, L, U, P, Q, R, Ns, Nt, Nx)
            % #TODO
        end
        
     	function y = PCStoredLU(x, Ca, Cg, L, U, P, Q, R, Ns, Nt, Nx)
            % #TODO
        end
        
        %% Configure
        function solverOut = configureSolver(varargin)
            if nargin > 0
                solverOut = TPFEM(varargin{:});
            end
        end
    end
end