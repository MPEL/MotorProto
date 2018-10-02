classdef HarmonicBalance < Solver
    properties
        GMRESTolerance  = 1e-3;
        NewtonTolerance = 1e-6;

        MaxGMRESIterations  = 100;
        MinNewtonIterations = 1;
        MaxNewtonIterations = 20;
        
        Adaptive               = false;
        SmoothingTolerance     = 1e-2;
        MinSmoothingIterations = 1;
        MaxSmoothingIterations = 2;
        AdaptiveTolerance      = 1e-2;
    end
    
    methods
        %% Constructor
        function this = HarmonicBalance(varargin)
            if nargin > 0
                for iArg = 1:2:nargin
                    this.(varargin{iArg}) = varargin{iArg+1};
                end
            end
        end
        
        %% Solve
        function solution = solve(this, model, x0)
            %% Setup Matrices
            getMatrix = DynamicMatrixFactory(model);
            this.Matrices = getMatrix;
            Nx = length(getMatrix.f(0));
            W = blkdiag(getMatrix.PostProcessing.SobolevA);
            W = blkdiag(getMatrix.PostProcessing.Reduced2Full).' * W * blkdiag(getMatrix.PostProcessing.Reduced2Full);
            
            %% Initialize
            t = model.getTimePoints(this.TimePoints);
            t = linspace(0,t(end),this.TimePoints/2 + 1);
            Nt = numel(t) - 1;
            Nh = ceil(Nt/2)-1;
            T  = t(end);
            
            %% Allocate
            tic
            if nargin < 3
                X = zeros(Nx, Nt);
            else
                X = fft(x0,[],2) / size(x0,2);
            end
                
            C = getMatrix.C(0,1,1);
            if this.StoreDecompositions
                G = cell(1,Nt);
                g = zeros(Nx,Nt);
                
                K = cell(1,Nt);
                f = zeros(Nx,Nt);

                for i = 1:Nt
                    K{i} = getMatrix.K(t(i),1);
                    f(:,i) = getMatrix.f(t(i),1);
                end
                
                M1 = cell(5,Nt);
                M2 = cell(5,Nt);
                
                MVP = @HarmonicBalance.MVPStoredLU;
                PC = @HarmonicBalance.PCStoredLU;
            else
            	MVP = @HarmonicBalance.MVPUnstored;
              	PC = @HarmonicBalance.PCUnstored;
            end
            
            %% Start Solve
           	first = true;
            if this.Adaptive
                last = false;
            else
                last = true;
            end
            
            aIter   = 1;
            discErr = 1;
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
                
                %% Subdivide Time Interval
                if discErr > this.AdaptiveTolerance
                    aIter = aIter + 1;
                    d = 2;

                    if mod(Nt,2) == 0 && d > 1
                        X = [X(:,1:Nh), X(:,Nh+1) / 2, zeros(Nx, (d-1)*Nt-1), conj(X(:,Nh+1)) / 2, X(:,(Nh+2):Nt)];
                    else
                        X = [X(:,1:(Nh+1)), zeros(Nx, (d-1)*Nt), X(:,(Nh+2):Nt)];
                    end
                    
                    Nt = d*Nt;
                    Nh = ceil(Nt/2)-1;
                    if mod(Nt,2) == 0
                        h = [0:Nh, 0, -Nh:-1] * 2 * pi / T;
                    else
                        h = [0:Nh, -Nh:-1] * 2 * pi / T;
                    end
                    t = linspace(0,T,Nt+1);
                    
                    if this.StoreDecompositions
                        G = [G, cell(1, Nt*(d-1)/d)];
                        g = [g, zeros(Nx, Nt*(d-1)/d)];

                        K = [K, cell(1, Nt*(d-1)/d)];
                        f = [f, zeros(Nx, Nt*(d-1)/d)];

                        for i = (Nt-d+1):-d:1
                            j      = (i+d-1) / d;
                            K{i}   = K{j};
                            f(:,i) = f(:,j);
                        end

                        for i = setdiff(1:Nt, 1:d:Nt)
                            K{i}   = getMatrix.K(t(i),1);
                            f(:,i) = getMatrix.f(t(i),1);
                        end
                        
                        M1 = [M1,cell(5,Nt*(d-1)/d)];
                        M2 = [M2,cell(5,Nt*(d-1)/d)];
                    end
                    
                    R = zeros(Nx,Nt);
                    
                    nIter = 1;
                    if this.Verbose
                        display(sprintf('Refining grid to %d time-points\n', Nt));
                    end
                else
                    last = true;
                end
                
                %% Calculate Residual Vector
                for i = 1:Nt
                    R(:,i) = 1i*h(i)*(C*X(:,i));
                end
                
                alpha = 0;
                beta  = 0;
                x = ifft(X,[],2,'symmetric') * Nt;
                r = ifft(R,[],2,'symmetric') * Nt;
                J0 = sparse(Nx,Nx);
                for i = 1:Nt
                    [Gi,gi] = getMatrix.G(t(i), x(:,i), 1);
                    
                    if this.StoreDecompositions
                        Ki = K{i};
                        fi = f(:,i);
                        G{i} = Gi;
                        g(:,i) = gi;
                    else
                        Ki = getMatrix.K(t(i),1);
                        fi = getMatrix.f(t(i),1);
                    end
                    
                    r(:,i) = r(:,i) + Ki * x(:,i) + gi;
                    
                    J0 = J0 + Ki + Gi;
                    
                    beta = beta + 0.5*norm(r(:,i)+fi)^2;
                    
                    r(:,i) = r(:,i) - fi;
                    r(:,i) = r(:,i);
                    
                    alpha = alpha + norm(r(:,i))^2;
                end
                J0    = J0 / Nt;
                alpha = sqrt(alpha);
                beta  = sqrt(beta);
                
                if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nIter == 1)
%                     newtonTol = this.NewtonTolerance;
%                     minNewton = this.MinNewtonIterations;
%                     maxNewton = this.MaxNewtonIterations;
                    last = true;
                end
                
                %% Start Newton Iteration
                while ((alpha > newtonTol * beta) && (nIter <= maxNewton)) || (nIter <= minNewton)
                    %% Solve Linearized Equation
                    if this.StoreDecompositions
                        %% Calculate and Factor both Preconditioners
                        for i = 1:Nt
                            [M1{3,i},M1{4,i},M1{2,i},M1{5,i},M1{1,i}] = lu(K{i}+G{i}+(Nt/T)*C);
                            [M2{3,i},M2{4,i},M2{2,i},M2{5,i},M2{1,i}] = lu(J0 + 1i*h(i)*C);
                        end
                        
                        %% GMRES Solve
                      	dX = gmres(MVP, r(:), this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r(:), K, G, C, M1, M2, h);
                        dX = PC(dX,K,G,C,M1,M2,h);
                        X  = X - dX;
                    else
                        %% GMRES Solve
                        dX = gmres(MVP, r(:), this.MaxGMRESIterations, this.GMRESTolerance, 1, [], [], r(:), x, t, J0, C, h, getMatrix);
                        dX = PC(dX,x,t,J0,C,h,getMatrix);
                        X  = X - dX;
                    end
                    
                    %% Calculate Residual Vector
                    for i = 1:Nt
                        R(:,i) = 1i*h(i)*(C*X(:,i));
                    end
                    
                    alpha = 0;
                    beta  = 0;
                    r = ifft(R,[],2,'symmetric') * Nt;
                    x = ifft(X,[],2,'symmetric') * Nt;
                    J0 = J0 * 0;
                    for i = 1:Nt
                        [Gi,gi] = getMatrix.G(t(i), x(:,i), 1);

                        if this.StoreDecompositions
                            Ki = K{i};
                            fi = f(:,i);
                            G{i} = Gi;
                            g(:,i) = gi;
                        else
                            Ki = getMatrix.K(t(i),1);
                            fi = getMatrix.f(t(i),1);
                        end

                        r(:,i) = r(:,i) + Ki * x(:,i) + gi;

                        J0 = J0 + Ki + Gi;

                        beta = beta + 0.5*norm(r(:,i)+fi)^2;

                        r(:,i) = r(:,i) - fi;

                        alpha = alpha + norm(r(:,i))^2;
                    end
                    J0     = J0 / Nt;
                    alpha  = sqrt(alpha);
                    beta   = sqrt(beta);
                    
                    Z = X;
                    Z(:,1) = 0;
                    for i = 2:(floor(Nh/2)+1)
                        Z(:,i)       = 0;
                        Z(:,end-i+2) = 0;
                    end
                    z = ifft(Z,[],2,'symmetric') * Nt;
                    
                    discErr = 0;
                    normErr = 0;
                    for i = 1:Nt
                        discErr = max(discErr,sqrt(z(:,i).'*W*z(:,i)));
                        normErr = max(normErr,sqrt(x(:,i).'*W*x(:,i)));
                    end
                    discErr = discErr / normErr;
                    
                    if this.Adaptive && (discErr < this.AdaptiveTolerance) && ~(first && nIter == 1)
%                         newtonTol = this.NewtonTolerance;
%                         minNewton = this.MinNewtonIterations;
%                         maxNewton = this.MaxNewtonIterations;
                        last = true;
                    end
                    
                    if this.Verbose
                        display(sprintf('Iteration %d, Discrete Residual = %0.3g, Tolerance = %0.3g, Discretization Error = %0.3g, Tolerance = %0.3g \n', nIter, alpha / beta, newtonTol, discErr, this.AdaptiveTolerance));
                    end
                    
                   	nIter = nIter + 1;
                end
                %this.getTimeMappingHarmonics(X,t,h,W);
                first = first && ~first;
            end
            this.SimulationTime = toc;
            this.DiscretizationError = discErr;

           	if this.Verbose
                display(sprintf('Simulation Time = %0.3g seconds\n', this.SimulationTime));
            end
            
            %% Post Processing
            x = ifft(X, [], 2, 'symmetric') * Nt;
            x_t = X;
            for i = 1:Nt
                x_t(:,i) = 1i*h(i)*x_t(:,i);
            end
            x_t = ifft(x_t, [], 2, 'symmetric') * Nt;
            
            x = [x, x(:,1)];
            x_t = [x_t, x_t(:,1)];
            
            x = mat2cell(x, Nx, ones(1,Nt+1));
            x_t = mat2cell(x_t, Nx, ones(1,Nt+1));
            
            [x, x_t] = getMatrix.doPostProcessing(x, x_t);
            this.Times = linspace(0,T,Nt+1);
            this.Y   = X;
            this.X   = x;
            this.X_t = x_t;
            
            solution = Solution(this);
        end
    end
    
    methods (Static)
    	function x = MVPStoredLU(x,K,G,C,M1,M2,h)
            Nt = length(G);
            Nx = length(x) / Nt;
            
            x = reshape(x,Nx,Nt);
            y = zeros(Nx,Nt);
            
            %% Time-Domain Part
            for i = 1:Nt
                y(:,i) = M1{5,i} * (M1{4,i} \ (M1{3,i} \ (M1{2,i} * (M1{1,i} \ x(:,i)))));
                x(:,i) = x(:,i) - (K{i}*y(:,i)) - (G{i}*y(:,i));
            end
            
            %% Frequency-Domain Part
            x = fft(x,[],2);
            y = fft(y,[],2);
            for i = 1:Nt
                x(:,i) = x(:,i)-1i*h(i)*(C*y(:,i)); % Frequency domain residual
                y(:,i) = y(:,i)+(M2{5,i} * (M2{4,i} \ (M2{3,i} \ (M2{2,i} * (M2{1,i} \ x(:,i)))))); % Apply second preconditioner, y = x1+x2
                x(:,i) = 1i*h(i)*(C*y(:,i)); % Frequency domain part of MVP
            end
            
            %% Time-Domain Part
            x = ifft(x,[],2,'symmetric');
            y = ifft(y,[],2,'symmetric');
            for i = 1:Nt
                x(:,i) = x(:,i) + (K{i}*y(:,i)) + (G{i}*y(:,i));
            end
            
            x = reshape(x,Nx*Nt,1);
        end
        
        function y = PCStoredLU(x,K,G,C,M1,M2,h)
            Nt = length(G);
            Nx = length(x) / Nt;
            
            x = reshape(x,Nx,Nt);
            y = zeros(Nx,Nt);
            
            %% Time-Domain Part
            for i = 1:Nt
                y(:,i) = M1{5,i} * (M1{4,i} \ (M1{3,i} \ (M1{2,i} * (M1{1,i} \ x(:,i)))));
                x(:,i) = x(:,i) - (K{i}*y(:,i)) - (G{i}*y(:,i));
            end
            
            %% Frequency-Domain Part
            x = fft(x,[],2) / Nt;
            y = fft(y,[],2) / Nt;
            for i = 1:Nt
                x(:,i) = x(:,i)-1i*h(i)*(C*y(:,i)); % Frequency domain residual
                y(:,i) = y(:,i)+(M2{5,i} * (M2{4,i} \ (M2{3,i} \ (M2{2,i} * (M2{1,i} \ x(:,i)))))); % Apply second preconditioner, y = x1+x2
            end
        end
        
       	function x = MVPUnstored(x,z,t,J0,C,h,getMatrix)
            [Nx,Nt] = size(z);
            T = t(end);
            
            x = reshape(x,Nx,Nt);
            y = zeros(Nx,Nt);
            
            %% Time-Domain Part
            for i = 1:Nt
                Ji = getMatrix.K(t(i), 1) + getMatrix.G(t(i), z(:,i), 1);
                y(:,i) = (Ji + Nt/T*C) \ x(:,i);
                x(:,i) = x(:,i) - Ji*y(:,i);
            end
            
            %% Frequency-Domain Part
            x = fft(x,[],2);
            y = fft(y,[],2);
            for i = 1:Nt
                x(:,i) = x(:,i)-1i*h(i)*(C*y(:,i));     % Frequency domain residual
                y(:,i) = y(:,i)+ (J0+1i*h(i)*C)\x(:,i); % Apply second preconditioner, y = x1+x2
                x(:,i) = 1i*h(i)*(C*y(:,i));            % Frequency domain part of MVP
            end
            
            %% Time-Domain Part
            x = ifft(x,[],2,'symmetric');
            y = ifft(y,[],2,'symmetric');
            for i = 1:Nt
                Ji = getMatrix.K(t(i), 1) + getMatrix.G(t(i), z(:,i), 1);
                x(:,i) = x(:,i) + Ji*y(:,i);
            end
            
            x = reshape(x, Nx*Nt, 1);
        end
        
        function y = PCUnstored(x,z,t,J0,C,h,getMatrix)
            [Nx, Nt] = size(z);
            T = t (end);
            
            x = reshape(x,Nx,Nt);
            y = zeros(Nx,Nt);
            
            %% Time-Domain Part
            for i = 1:Nt
                Ji = getMatrix.K(t(i), 1) + getMatrix.G(t(i), z(:,i), 1);
                y(:,i) = (Ji + Nt/T*C) \ x(:,i);
                x(:,i) = x(:,i) - Ji*y(:,i);
            end
            
            %% Frequency-Domain Part
            x = fft(x,[],2) / Nt;
            y = fft(y,[],2) / Nt;
            for i = 1:Nt
                x(:,i) = x(:,i) - 1i*h(i)*(C*y(:,i));    % Frequency domain residual
                y(:,i) = y(:,i) + (J0+1i*h(i)*C)\x(:,i); % Apply second preconditioner, y = x1+x2
            end
        end
        
        function [s, l_t, L, L_t, k] = getTimeMappingHarmonics(X, t, h, W)
            T  = t(end)-t(1);
            M  = 10;
            Nt = length(h);
            X_t = X;
            for i = 1:Nt
                X_t(:,i) = (1i*h(i))^2*X_t(:,i);
            end
            
            %% Calculate Time-Map Derivative Harmonics
            l_t = zeros(1,M*Nt);
            for i = 1:(M*Nt)
                ti = (i-1) / (M*Nt) * T;
                x_t = real(X_t*exp(1i*h.'*ti));
                l_t(i) = 1/sqrt(x_t.'*W*x_t);
            end
            L_t = fft(l_t,[],2) / (M*Nt);
            L_t = L_t / L_t(1);
            L_t([1 end/2+1]) = 0;
            
            %% Calculate Time-Map Harmonics
            k = 1i*2*pi/T*[0:(M*Nt/2-1) 0 (-M*Nt/2+1):-1];
            L = L_t ./ k;
            L([1 end/2+1]) = 0;
            
            %% Calculate 2Nt Time Points Corresponding to Equal Spaced Pseudo-Times
            s = zeros(1,2*Nt+1);
            for i = 1:(2*Nt+1)
                tau_i = (i-1)*T/(2*Nt);
                if i > 1
                    s(i) = s(i-1) + T/(2*Nt);
                else
                    s(i) = tau_i;
                end
                r = tau_i - s(i) - real(L*exp(k.'*s(i)));
                while abs(r) > T/(2*Nt)*sqrt(eps)
                    J = -1-real(L_t*exp(k.'*s(i)));
                    s(i) = s(i) - J \ r;
                    r = tau_i - s(i) - real(L*exp(k.'*s(i)));
                end
            end
            
            l_t = 1+real(L_t*exp(k.'*s));
        end
        
        function solverOut = configureSolver(varargin)
            if nargin > 0
                solverOut = HarmonicBalance(varargin{:});
            end
        end
    end
end