classdef Static < Solver
    %Static.m The standard solver for magnetostatic problems.
    %   Static is a solver for periodic magnetostatic problems. For a given
    %   model, a series of magnetostatic simulations are performed at a
    %   number of time points over the problem period.
    %
    % Static properties:
    %   NewtonTolerance     - Sets the relative residual tolerance
    %   MaxNewtonIterations - Sets the maximum number of iterations
    %   TimeInterval        - Optionally specifies a time window over which to perform analysis
    
    % Static inherits properties and methods Solver.
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
        MaxNewtonIterations = 100;
        TimeInterval        = [];
    end
    
    methods
        function this = Static(varargin)
            if nargin > 0
                for iArg = 1:2:nargin
                    this.(varargin{iArg}) = varargin{iArg+1};
                end
            end
        end
        
        function solution = solve(this, model, x0)
            %% Configure matrices
            matrixFactory = DynamicMatrixFactory(model);
            this.Matrices = matrixFactory;
            
            %% Configure algorithm
          	maxNIt  = this.MaxNewtonIterations;
            nrrTol  = this.NewtonTolerance;
            verbose = this.Verbose;
            
            %% Get times points
            if isempty(this.TimeInterval)
                t = model.getTimePoints(this.TimePoints);
            else
                t = linspace(this.TimeInterval(1),this.TimeInterval(2),this.TimePoints);
            end

            this.Times = t;
            Nt         = numel(t);
            x          = cell(1, Nt);
            
            %% For each time point, 
            tic
            for i = 1:Nt
                %% Create constant matrices,
                f = matrixFactory.f(t(i));
                K = matrixFactory.K(t(i));

                %% Get initial guess,
                if i > 1
                    x{i} = x{i-1};
                else
                    if nargin == 3
                        x{1} = x0;
                    else
                        x{i} = zeros(size(f));
                    end
                end
                
                %% Perform Newton-Raphson iteration, 
                nIter  = 1;
                relRes = 1;
                
                if verbose
                    Time = i
                end
                
                while nIter < maxNIt && relRes > nrrTol
                    [G, g] = matrixFactory.G(t(i), x{i});
                    
                    r = K * x{i} + g - f;
                    J = K + G;
                    
                    dx = J \ r;
                    
                    x{i} = x{i} - dx;
                    
                    nIter  = nIter + 1;
                    relRes = norm(r) / norm(g-f);
                end
            end
            this.SimulationTime = toc;
            
            if verbose
                SimulationTime = this.SimulationTime
            end
            
            %% Estimate Derivatives
            if isempty(this.TimeInterval)
                x_t = x(:, 1:(Nt - 1));
                x_t = cell2mat(x_t);
                
                hMax         = (Nt - 1)/2 - 1;
                h            = [1:(hMax+1), (hMax+3):(2*hMax + 2)];
                nyqEl        = hMax + 2;
                I            = 1:(2*hMax + 1);
                fe           = 1 / t(end);
                omega        = 1i*2*pi*fe * sparse(I, I, [0:1:hMax -hMax:1:-1]);
                x_t          = fft(x_t, [], 2);
                x_t(:,h)     = x_t(:, h) * omega;
                x_t(:,nyqEl) = 0;
                x_t          = ifft(x_t, [], 2);
                x_t          = mat2cell(x_t, length(x_t(:,1)), ones(1, Nt - 1));
                x_t(:,end+1) = x_t(:, 1);
            else
                x   = cell2mat(x);
                x_t = x;
                Nu  = size(x,1);
                t   = repmat(t,Nu,1);
                
                p = polyfit(t(:,1:3),x(:,1:3),2);
                x_t(:,1) = p(:,1)*t(1,1)^2 + p(:,2)*t(1,1) + p(:,3);
                x_t(:,2) = p(:,1)*t(1,2)^2 + p(:,2)*t(1,2) + p(:,3);

                for i = 3:(Nt-1)
                    I = (i-1):(i+1);
                    p = polyfit(t(:,I), x(:,I),2);
                    x_t(:,i) = p(:,1)*t(1,I(2))^2 + p(:,2)*t(1,I(2)) + p(:,3);
                end
                
                x_t(:,Nt) = p(:,1)*t(1,Nt)^2 + p(:,2)*t(1,Nt) + p(:,3);
                
                x_t = mat2cell(x_t, Nu, ones(1, Nt));
                x   = mat2cell(x, Nu, ones(1, Nt));
            end
            
            %% Save Solution
            [x, x_t] = matrixFactory.doPostProcessing(x, x_t);
            this.X   = x;
            this.X_t = x_t;
            solution = Solution(this);
        end
    end
    
    methods (Static)        
        function solverOut = configureSolver(varargin)
            if nargin > 0
                solverOut = Static(varargin{:});
            end
        end
    end
end