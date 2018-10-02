function MBFit(M,B)
    %% Main Interpolation Function
    vars   = {'M','B','p1','p2','p3','p4'};
    M1     = sym('p1*(p2 + (p3*p4)*exp(-B/p3) - (2*p3*p4*atan(exp((p2 - B)/p3)))*exp(-p2/p3) - p3*log(1 + exp((2*(p2 - B))/p3)))/(1-tanh(-p2/p3))');
    M1     = simple(M1);
    M1     = simple(M1 - subs(M1,'B',0));

    m1Fun  = matlabFunction(M1,'vars',vars(2:end))
    
    F1     = (sym('M')-M1)^2/(sym('M^2'))
    f1Fun  = matlabFunction(F1,'vars',vars)
    
    G1     = jacobian(F1,sym('[p1,p2,p3,p4]')).'
    g1Fun  = matlabFunction(G1,'vars',vars)
    
    H1     = jacobian(G1,sym('[p1,p2,p3,p4]'))
    h1Fun  = matlabFunction(H1,'vars',vars)

    [p1,p2,p3,p4] = getInitialGuess(f1Fun,M(2:end),B(2:end))
    
	[p1,p2,p3,p4] = minimize(f1Fun,g1Fun,h1Fun,M(2:end),B(2:end),p1,p2,p3,p4)
    
    BTest = linspace(0,max(B)*2);
    figure;clf
    scatter(B,M);hold on;
    plot(BTest,m1Fun(BTest,p1,p2,p3,p4),'-x');
    title('M-B Interpolation');
    
    figure;clf
    scatter(B/mu_o-M,B);hold on;
    plot(BTest/mu_o-m1Fun(BTest,p1,p2,p3,p4),BTest,'-x');
    title('B-H Interpolation');
    
    DM = M-m1Fun(B,p1,p2,p3,p4);
    figure;clf
    scatter(B,DM);
    title('M1 Magnetization Error');    
    
    figure;clf
    scatter(B,abs(DM./M)*100);
    title('Magnetization Error (Relative to M)');
    ylabel('Percent Error')
    
    figure;clf
    scatter(B,abs(DM./(B/mu_o-M))*100);
    title('Magnetization Error (Relative to H)');
    ylabel('Percent Error')
end

% -c(1)*c(3)*(log(exp(-2*(B-c(2))/c(3))+1)-c(4)*exp(-B/c(3))+c(4)*exp(-c(2)/c(3))*atan(exp(-(B-c(2))/c(3)))*2)
% 
% -c(1)*c(3)*(c(4)-log(exp(c(2)*2/c(3))+1)-c(4)*exp(-c(2)/c(3))*atan(exp(c(2)/c(3)))*2)

function [p1,p2,p3,p4] = getInitialGuess(fIn,M,B)
    N = 21;
    p1 = linspace(min(M./B),max(M./B),N);
    p2 = linspace(1.6,1.9,N);
    p3 = linspace(0,0.3,N);
    p4 = linspace(0,0.001,N);
    f  = zeros(N,N,N,N);
    
    for i = 1:N
        for j = 1:N
            for k = 1:N
                for l = 1:N
                    f(i,j,k,l) = sum(fIn(M,B,p1(i),p2(j),p3(k),p4(l)));
                end
            end
        end
    end
    
    [f,l] = min(f,[],4);
    [f,k] = min(f,[],3);
    [f,j] = min(f,[],2);
    [f,i] = min(f,[],1);
       j  = j(i);
       k  = k(i,j);
       l  = l(i,j,k);
       p1 = p1(i);
       p2 = p2(j);
       p3 = p3(k);
       p4 = p4(l);
end

function varargout = minimize(fIn,gIn,hIn,M,B,varargin)
    varargout = varargin;
    n         = numel(varargout);
    dF        = inf;
    itterMax  = 1000;
    fOut      = zeros(itterMax,1);
    fOut(1)   = objectiveFunction(fIn,M,B,varargin{:});
    i         = 1;
    while i < itterMax && dF > sqrt(eps)
        gOut     = objectiveGradient(gIn,M,B,varargout{:});
        hOut     = objectiveHessian(hIn,M,B,varargout{:});
        sOut     = diag(1./sum(hOut))*0+eye(size(hOut));
        
        dParam   = sOut*(hOut*sOut)\gOut;
        
        if any(isnan(dParam))
            i
            break
        end
        
        for j = 1:n
            varargout{j} = varargout{j}-dParam(j);
        end
        
        if varargout{end} < 0
            varargout{end} = 0;
        end
        
        i        = i+1;
        fOut(i)  = objectiveFunction(fIn,M,B,varargout{:});
        dF       = abs(fOut(i)-fOut(i-1))/(fOut(i)+sqrt(eps));
    end
    figure;clf
    semilogy(fOut);
    legend('Convergence Plot');
end

function fOut = objectiveFunction(fIn,M,B,varargin)
    fOut = 0;
    
    n    = numel(M);
    
    for i = 1:n
        fOut = fOut + fIn(M(i),B(i),varargin{:});
    end
end

function gOut = objectiveGradient(gIn,M,B,varargin)
    m    = numel(varargin);
    gOut = zeros(m,1);
    
    n    = numel(M);
    
    for i = 1:n
        gOut = gOut + gIn(M(i),B(i),varargin{:});
        if any(isnan(gOut))
            break
        end
    end
end

function hOut = objectiveHessian(hIn,M,B,varargin)    
    m    = numel(varargin);
    hOut = zeros(m,m);
    
    n    = numel(M);
    
    for i = 1:n
        hOut = hOut + hIn(M(i),B(i),varargin{:});
    end
end

