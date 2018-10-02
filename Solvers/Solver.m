classdef Solver
    %Solver.m Base class for all solvers.
    %
    % Solver properties:
    %   Matrices           - MatrixFactory objects used by the solver
    %   TimePoints         - Target number of simulation time points
    %   Times              - The simulation times used by the solver
    %   Verbose            - Toggles verbose simulation progress reports
    %   X                  - The solution produced by the solver
    %   X_t                - Time derivative of the solver solution
    %   SimulationTime     - Total simulation time
    %   ConvergenceHistory - Relative residual history
    %
    % Solver methods:
    %   solve - Run the simulation
    %
    % See also MotorProto
    
%{
properties:
 	%TimePoints - Target number of simulation time points
 	%	The TimePoints property is an integer that sets the target number
 	%   of time points used in the simulation. This number is not adhered
 	%   to exactley but used as a guide to determine an appropriate number
 	%   of time points to use after considering harmonic content of the
 	%   solution and the effects of aliasing. In general, the number of
 	%   time points used will be greater than that specified in the
 	%   TimePoints, but not much larger.
    %
    % See also Solver, MatrixFactory/getTimePoints
    TimePoints;
%}

    properties (SetAccess = protected)
        Matrices
    end
    
    properties
        TimePoints
        Times
        
        StoreDecompositions = false;
        Verbose = true;
    end
    
    properties (Dependent, SetAccess = private)
        Harmonics
    end
    
    properties (SetAccess = protected)
        Y
        Y_t
        
        X
        X_t
        
     	SimulationTime
        DiscretizationError
    end
    
    methods (Static)        
        function solverOut = configureSolver(solverType,varargin)
            solverOut = eval(solverType);
            if isa(solverOut,'Solver')
                if nargin > 1;
                    solverOut = solverOut.configureSolver(varargin{:});
                end
            else
                error('MotorProto:Solver:invalidObjectType', '%s is not a recognized Solver subclass',solverType);
            end
        end
        
     	function [a,b,c,d,p,q,bu,be,pe] = getButcherTable(Ns)
            switch Ns
                case 1
                    a = [0 0;
                         0 1];
                    bu = [0;1];
                    be = [0,1];
                    pe = 1;
                case 2
                    a = [0,             0,             0;
                         1 - 2^(1/2)/2, 1 - 2^(1/2)/2, 0;
                         2^(1/2)/4,     2^(1/2)/4,     1 - 2^(1/2)/2];
                    
                    bu = [sqrt(2)/2, -sqrt(2)/4;
                          sqrt(2)/2, -sqrt(2)/4;
                          1-sqrt(2), sqrt(2)/2];
                    be = [-sqrt(2)/4, -sqrt(2)/4, sqrt(2)/2];
                    pe = 2;
                case 3            
                    c2 = roots([3 -18 18 -4]);
                    c2 = c2(3);
                    c2 = c2-((((c2-6)*c2+6)*c2)/(3*((c2-4)*c2+2))-4/(9*((c2-4)*c2+2)));
                    
                    a = zeros(4,4);
                    a(2,1) = c2 / 2;
                    a(2,2) = c2 / 2;
                    a(3,1) = (c2*(c2*(c2*(c2*(9*c2 - 30) + 38) - 20) + 4)) / (c2*(c2*(c2*(18*c2 - 48) + 56) - 32) + 8);
                    a(3,2) = (-c2*(c2*(c2*(3*c2 - 9) + 10) - 4)) / (c2*(c2*(c2*(9*c2 - 24) + 28) - 16) + 4);
                    a(3,3) = c2 / 2;
                    a(4,1) = (-c2*(c2*(c2*(9*c2 - 27) + 31) - 14) - 2) / (c2*(c2*(c2*(c2*(9*c2 - 54) + 102) - 84) + 24));
                    a(4,2) = (c2*(c2*(9*c2 - 30) + 34) - 12) / (c2*(c2*(12*c2 - 60) + 72) - 24);
                    a(4,3) = (-c2*(c2*(c2*(c2*(c2*(27*c2 - 108) + 198) - 208) + 132) - 48) - 8) / (c2*(c2*(c2*(c2*(c2*(36*c2 - 252) + 624) - 744) + 432) - 96));
                    a(4,4) = c2 / 2;
                    
                    bu = zeros(4,3);
                    bu(1,3) = -((3*c2^3 - 8*c2^2 + 10*c2 - 4))/(3*(3*c2^3 - 6*c2^2 + 4*c2)*(c2^2 - 4*c2 + 2));
                    bu(1,2) = ((9*c2^4 - 21*c2^3 + 15*c2^2 + 6*c2 - 6))/(3*(3*c2^3 - 6*c2^2 + 4*c2)*(c2^2 - 4*c2 + 2));
                    bu(1,1) = ((- 18*c2^4 + 51*c2^3 - 54*c2^2 + 18*c2))/(3*(3*c2^3 - 6*c2^2 + 4*c2)*(c2^2 - 4*c2 + 2));
                    
                    bu(2,3) = ((6*c2^2 - 20*c2 + 12))/(12*(c2 - 1)*(c2^2 - 4*c2 + 2));
                    bu(2,2) =  ((-9*c2^3 + 18*c2^2 + 6*c2 - 12))/(12*(c2 - 1)*(c2^2 - 4*c2 + 2));
                    bu(2,1) = ((18*c2^3 - 54*c2^2 + 48*c2 - 12))/(12*(c2 - 1)*(c2^2 - 4*c2 + 2));
                    
                    bu(3,3) = ((2*c2 - 4)*(3*c2^2 - 4*c2 + 2)^2)/(12*(c2^2 - 4*c2 + 2)*(- 3*c2^4 + 9*c2^3 - 10*c2^2 + 4*c2));
                    bu(3,2) = - ((3*c2^2 - 6)*(3*c2^2 - 4*c2 + 2)^2)/(12*(c2^2 - 4*c2 + 2)*(- 3*c2^4 + 9*c2^3 - 10*c2^2 + 4*c2));
                    bu(3,1) = - ((6*c2 - 6*c2^2)*(3*c2^2 - 4*c2 + 2)^2)/(12*(c2^2 - 4*c2 + 2)*(- 3*c2^4 + 9*c2^3 - 10*c2^2 + 4*c2));
                    
                    bu(4,3) = (2/(3*(c2^2 - 4*c2 + 2)));
                    bu(4,2) = (-(2*c2)/(c2^2 - 4*c2 + 2));
                    bu(4,1) = (c2^2/(c2^2 - 4*c2 + 2));
                    
                    be = zeros(1,4);
                    be(1) = -((c2 - 1)^2*(3*c2^2 - 4*c2 + 2))/(c2*(3*c2^2 - 6*c2 + 4)*(c2^2 - 4*c2 + 2));
                    be(2) = ((3*c2^2)/4 - 1/2)/((c2 - 1)*(c2^2 - 4*c2 + 2)) + 3/4;
                    be(3) = ((3*c2^2 - 4*c2 + 2)^2*(c2^2 - 2*c2 + 2))/(4*(c2^2 - 4*c2 + 2)*(- 3*c2^4 + 9*c2^3 - 10*c2^2 + 4*c2));
                    be(4) = (c2*(c2 - 2))/(c2^2 - 4*c2 + 2);
                    be = a(end,:) - be;
                    pe = 3;
            end
            b = a(end,:);
            c = sum(a,2);
            c = c(2:end);
            q = a(2:end,2:end) \ a(2:end,1);
            
            assert(abs(q(end)) < sqrt(eps));
            q(end) = 0;
            
            a = a(2:end,2:end);
            d = inv(a);
            p = sum(d,2);
        end
        
        function [z, z_t, s] = rkInterpolate(t, y, y_t, c, bu, s)
            [Rs, ~] = size(y);
            [m,n]   = size(bu);
            T       = t(end) - t(1);
            Nt      = length(t) - 1;
            
            Ns  = length(s) - 1;
            z   = cell(Rs, Ns);
            z_t = cell(Rs, Ns);
            for i = 1:Ns
                ds = s(i+1) - s(i);
                for j = 1:Rs
                    s_ij = s(i) + ds * c(j);
                    s_ij = mod(s_ij-T,T) + T;
                    s_ij = mod(s_ij-t(1),t(end)-t(1)) + t(1);
                    
                    k = find(t > s_ij, 1, 'first') - 1;
                    km1 = mod(k-2,Nt) + 1;
                    h_k = t(k+1)-t(k);
                    
                    u = (s_ij - t(k)) / h_k;

                    z{j,i} = y{Rs,km1};
                    z_t{j,i} = 0 * z{j,i};
                    for jj = 1:n
                        z{j,i} = z{j,i} + h_k * bu(1,jj) * y_t{Rs,km1} * u^jj;
                        z_t{j,i} = z_t{j,i} + bu(1,jj) * y_t{Rs,km1} * (jj*u^(jj-1)); 
                    end
                    
                    for ii = 2:m
                        for jj = 1:n
                            z{j,i} = z{j,i} + h_k * bu(ii,jj) * y_t{ii-1,k} * u^jj;
                            z_t{j,i} = z_t{j,i} + bu(ii,jj) * y_t{ii-1,k} * (jj*u^(jj-1)); 
                        end
                    end
                end
            end
        end
        
        function [ec, emax] = rkErrorCoefficients(t, y, y_t, be, pe, getMatrix)
            Nt = numel(t)-1;
            Ns = length(be) - 1;
            
            W = blkdiag(getMatrix.PostProcessing.SobolevA);
            W = blkdiag(getMatrix.PostProcessing.Reduced2Full).' * W * blkdiag(getMatrix.PostProcessing.Reduced2Full);
            
            ec = zeros(1,Nt);
            Cn = 0;
            for k = 1:Nt
                km1 = mod(k-2,Nt) + 1;
                h_k = t(k+1) - t(k);
                
                ep = be(1) * y_t{end,km1};
                for i = 1:Ns
                   ep = ep + be(i+1) * y_t{i,k};
                end
                ep    = ep * h_k^(1-pe);
                ec(k) = sqrt(ep.'*W*ep);
                
                Cn  = max(Cn, sqrt(y{end,k}.'*W*y{end,k}));
            end
            ec = (ec ./ Cn);
            ec(1:(end/2)) = ec((end/2+1):end);
            ec = [ec(end),ec];
            h  = [t(end)-t(end-1), diff(t)];
            
            emax = max(ec.*h.^pe)^((pe+1)/pe);
            
%             figure;
%             s = [t(1:end-1);t(2:end)];
%             s = reshape(s,1,[]);
%             v = (ec);
%             v = [v(1:end-1);v(1:end-1)];
%             v = reshape(v,1,[]);
%             plot(s,v)
%             pause(1);
%             figure;
%             plot(t, ec)
%             pause(1);
        end
        
        function [s, tol] = rkRefine(t, tol, tol_max, ec, pe, rfact, init)
            tol_max = tol_max^(pe/(pe+1));
            
            if init
                h   = [t(end)-t(end-1),diff(t)];
                tol = mean(ec.*h.^pe);
                ngr = floor((-1/pe) * log(tol_max / tol) / log(rfact));
                %tol = 2^(-1/(pe+1))*tol_max * (2^(pe*ngr));
                tol = 0.5 * tol_max * (2^(pe*ngr));
            else
                if tol*2^(-pe) >= 0.5 * tol_max
                    tol = tol * 2^(-pe);
                else
                    tol = tol * 2^(-1/(pe+1));
                end
            end
            
            Nt = length(t) - 1;
            T  = t(end) - t(1);
            
            if Nt < 12
                s = linspace(0,T,2*Nt+1);
            else
                %% Calculate new time-point function
                hc = (tol./ec).^(1/pe);
                
%                 s = [t(1:end-1);t(2:end)];
%                 v = [hc(2:end);hc(2:end)];
%                 figure;
%                 plot(reshape(s,1,[]),reshape(v,1,[]));
%                 pause(1);
                
                I  = 2:(Nt/6+1);
                hc(I) = hc(I+3*Nt/6); %values in [T/2,T] are usually more well smoothed
                for i = 4:5
                    hc(I) = min(hc(I), hc(I+i*Nt/6));
                end

                for i = 1:5
                    hc(I+i*Nt/6) = hc(I);
                end
                hc(end) = hc(1);

                %% Calculate New Time Points
%                 t1 = t(1:(end-1));
%                 t2 = t(2:end);
% 
%                 minax = [];
%                 s = [];
%                 imin = [];
%                 imax = [];
%                 for i = 2:(Nt-1)
%                     j = i-1;
%                     k = i+1;
%                     if (hc(i) <= hc(j)) && (hc(i) < hc(k))
%                         s     = cat(2,s,t(i));
%                         minax = cat(2,minax,-1);
%                         imin  = cat(2,imin,i);
%                     elseif (hc(i) >= hc(j)) && (hc(i) > hc(k))
%                         s     = cat(2,s,t(i));
%                         minax = cat(2,minax,1);
%                         imax  = cat(2,imax,i);
%                     end
%                 end
%                 I = find(s > 0);
%                 if minax(I(1)) == -1
%                     I(1) = [];
%                 end
%                 s = s(I);
%                 minax = minax(I);
%                 
%                 I = (s <= s(1) + T/6*(1+sqrt(eps)));
%                 s = s(I);
%                 minax = minax(I);
%                 assert(minax(end) == 1);
%                 
%                 m = length(s);
%                 
%                 for k = 1:2:m
%                     if k < m %forward from max s(k) to min s(k+1)
%                         sf = [];
%                         sk = s(k);
%                         while sk < s(k+1)
%                             j = find((t1 <= sk) & (sk < t2)) + 1;
%                             i = j;
%                             while t(i)-sk  < hc(i)
%                                 i = i + 1;
%                             end
%                             if t(i-1)-sk > hc(i)
%                                 i = i - 1;
%                             end
%                             hf = min(hc(j:i));
%                             sk = sk + hf;
%                             sf = cat(2,sf,sk);
%                         end
%                         sf = (sf-s(k)) * (s(k+1)-s(k)) / (sf(end)-s(k)) + s(k);
%                         sf(end) = [];
%                         s = cat(2,s,sf);
%                     end
%                     
%                     if k > 1 %backward from max s(k) to min s(k-1)
%                         sb = [];
%                         sk = s(k);
%                         while sk > s(k-1)
%                             j = find((t1 < sk) & (sk <= t2)) + 1;
%                             i = j;
%                             while sk-t(i-1) < hc(i)
%                                 i = i - 1;
%                             end
%                             if sk-t(i) > hc(i)
%                                 i = i + 1;
%                             end
%                             hb = min(hc(i:j));
%                             sk = sk - hb;
%                             sb = cat(2,sb,sk);
%                         end
%                         sb = (sb - s(k)) * (s(k) - s(k-1)) / (s(k)-sb(end)) + s(k);
%                         sb(end) = [];
%                         s = cat(2,s,sb);
%                     end
%                 end
                
                t1 = t(1:(end-1));
                t2 = t(2:end);
                interval = zeros(0,2);
                minax    = zeros(0,2);
                
                for i = 2:(Nt-1)
                    j = i-1;
                    k = i+1;
                    if (hc(i) <= hc(j)) && (hc(i) < hc(k))
                        if ~isempty(interval) && (interval(end,2) ~= j)
                            interval(end+1,:) = [interval(end,2),j];
                            minax(end+1,:)    = [1,-1];
                        end
                        interval(end+1,:) = [j,i];
                        minax(end+1,:)    = [-1,-1];
                    elseif (hc(i) >= hc(j)) && (hc(i) > hc(k))
                        if ~isempty(interval)
                            interval(end+1,:) = [interval(end,2),i];
                            minax(end+1,:)    = [minax(end,2),1];
                        end
                    end
                end
                
                s = t(unique(interval));
                for l = 1:size(interval,1)
                    k1 = interval(l,1);
                    k2 = interval(l,2);
                    if minax(l,1) >= minax(l,2) %forward from max t(k1) to min t(k2)
                        sf = [];
                        sk = t(k1);
                        while sk < t(k2)
                            j = find((t1 <= sk) & (sk < t2)) + 1;
                            i = j;
                            while t(i)-sk  < hc(i)
                                i = i + 1;
                            end
                            if t(i-1)-sk > hc(i)
                                i = i - 1;
                            end
                            hf = min(hc(j:i));
                            sk = sk + hf;
                            sf = cat(2,sf,sk);
                        end
                        sf = (sf-t(k1)) * (t(k2)-t(k1)) / (sf(end)-t(k1)) + t(k1);
                        sf(end) = [];
                        s = cat(2,s,sf);
                    else %backward from max t(k2) to min t(k1)
                        sb = [];
                        sk = t(k2);
                        while sk > t(k1)
                            j = find((t1 < sk) & (sk <= t2)) + 1;
                            i = j;
                            while sk-t(i-1) < hc(i)
                                i = i - 1;
                            end
                            if sk-t(i) > hc(i)
                                i = i + 1;
                            end
                            hb = min(hc(i:j));
                            sk = sk - hb;
                            sb = cat(2,sb,sk);
                        end
                        sb = (sb - t(k2)) * (t(k2) - t(k1)) / (t(k2)-sb(end)) + t(k2);
                        sb(end) = [];
                        s = cat(2,s,sb);
                    end
                end
                s = sort(s);
                I = (s <= (s(1) + T/6*(1+sqrt(eps))));
                s = s(I);
                
                %% Copy
                s = sort(s);
                s(end) = [];
                s = [s,s+T/6,s+T/3,s+T/2,s+2*T/3,s+5*T/6];
                s = mod(s,T);
                s = sort(s);
                s = [s,s(1)+T];
            end
        end
    end
    
    methods
        function value = get.Harmonics(this)
            if ~isempty(this.Matrices)
                value = this.Matrices.getHarmonics(this.Times);
            else
                hMax = floor(numel(this.Times) / 2);
                if hMax > 0
                    value = [0:hMax -hMax:1:-1];
                else
                    value = [];
                end
            end
        end
        
        function X = fft(this, x)
            t = this.Times;
            dt = diff(t);
            if all(dt-mean(dt)) < sqrt(eps) * mean(dt)
                X = fft(x,[],2) / (numel(t) - 1);
            else
                %% Time--Mapped Fourier Transform
                T = t(end);
                N = numel(t) - 1;
                s = t / T;
                s = s-linspace(0,1,N+1);
                s(end) = [];
                S = fft(s,[],2) / N;
                
                M = 4 * N;
                lpart = linspace(0,1,M+1);
                lpart(end) = [];
                
                tr = lpart;
                tp = lpart;
                
                for i = 1:length(tp)
                    r = inf;
                    while abs(r) > T * sqrt(eps)
                        if mod(N,2) == 0
                            K = [0:(N/2) (1-N/2):-1];
                            D = exp(1i*K.'*2*pi*tp(i));
                            J = (1i*K.'*2*pi).*D;
                            D(N/2+1) = cos(2*pi*N/2*tp(i));
                            J(N/2+1) = -2*pi*N/2*sin(2*pi*N/2*tp(i));
                            J = S*J+1;
                        else
                            K = [0:((N-1)/2), ((1-N)/2):-1];
                            D = exp(1i*K.'*2*pi*tp);
                        end

                        r = tp(i)+S*D-tr(i);
                        d = J \ r;
                        tp(i) = tp(i) - d;
                    end
                    
                    if i < length(tp)
                        tp(i+1) = tp(i) + 1 / M;
                    end
                end
                tp = real(tp);
                
                X = fft(x,[],2) / N;
                if mod(N,2) == 0
                    K = [0:(N/2) (1-N/2):-1];
                    D = exp(1i*K.'*2*pi*tp);
                    D(N/2+1,:) = cos(2*pi*N/2*tp);
                else
                    K = [0:((N-1)/2), ((1-N)/2):-1];
                    D = exp(1i*K.'*2*pi*tp);
                end
                y = X*D;
                X = fft(y,[],2) / M;
                
                if mod(N,2) == 0
                    X = [X(:,1:(N/2)) X(:,(end-N/2+1):end)];
                    X(:,N/2+1) = 2*real(X(:,N/2+1));
                else
                    X = [X(1:((N-1)/2)) X((end-(N-1)/2+1):end)];
                end
            end
    	end
    end
    
    methods (Sealed)
        function copyOut = copy(this)
            nThis   = numel(this);
            copyOut = this;
            for i = 1:nThis
                copyOut(i).Matrices = copy(this.Matrices);
            end
        end
    end
    
    methods (Abstract)
        solution = solve(this, model, x0);
    end
end