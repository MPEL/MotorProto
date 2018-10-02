function [Bs,Bt,c] = mbfit(b,h)
    if size(b,1) == 1
        b = b.';
        h = h.';
    end
    
    if b(1) == 0
        b(1) = [];
        h(1) = [];
    end
    m = b/mu_o - h;
    
    %% Simple Search to Find Good Initial Condition
    fmin = inf;
    Bsmin = 0;
    Btmin = 0;
    for Bs = linspace(1,2.5,31)
        for Bt = 2.^(-(1:10))
            if exp(-Bs/Bt) > 0
                %% Optimize Basis Function Coefficients
                y      = Phi(b,Bt,Bs);
                lsqmat = y;
                weight = diag(1./h.^2);

                matinv = (lsqmat.'*weight*lsqmat);
                if cond(matinv) < 1/eps
                    c = matinv \ (lsqmat.' * weight * m);

                    %% Evaluate Objective Function
                    y    = M(b,Bt,Bs,c);
                    func = 0.5 * sum((y-m).^2 ./ h.^2);

                    if fmin > func
                        fmin  = func;
                        Bsmin = Bs;
                        Btmin = Bt;
                    end
                end
            end
        end
    end
    Bs     = Bsmin;
    Bt     = Btmin;
    y      = Phi(b,Bt,Bs);
    lsqmat = y;
    weight = diag(1./h.^2);
    c      = matinv \ (lsqmat.' * weight * m);
    
    for i = 1:2
        maxdir = inf;
       	grad = zeros(2+length(c),1);
        hess = zeros(2+length(c));
        while maxdir > sqrt(eps)
            %% Objective Function Evaluation
            y    = M(b,Bt,Bs,c);
            func = 0.5 * sum((y-m).^2 ./ h.^2);

            %% Gradient Evaluation
            dydBs = dMdBs(b,Bt,Bs,c);
            dydBt = dMdBt(b,Bt,Bs,c);
            
            grad(1) = sum((y-m).*dydBs ./ h.^2);
            grad(2) = sum((y-m).*dydBt ./ h.^2);

            for j = 1:numel(c)
                dydPj = dMdPhij(b,Bt,Bs,j);
                grad(j+2) = sum((y-m).*dydPj ./ h.^2);
            end
            
            %% Hessian Evaluation
            dy2dBs2   = dM2dBs2(b,Bt,Bs,c);
            dy2dBt2   = dM2dBt2(b,Bt,Bs,c);
            dy2dBtdBs = dM2dBtdBs(b,Bt,Bs,c);

            hess(1,1) = sum(((y-m).*dy2dBs2 + dydBs.^2) ./ h.^2);
            hess(2,2) = sum(((y-m).*dy2dBt2 + dydBt.^2) ./ h.^2);
            hess(1,2) = sum(((y-m).*dy2dBtdBs + dydBt.*dydBs) ./ h.^2);
            hess(2,1) = hess(1,2);

            for j = 1:numel(c)
                dydPj     = dMdPhij(b,Bt,Bs,j);
                dy2dBtdPj = dM2dBtdPhij(b,Bt,Bs,j);
                dy2dBsdPj = dM2dBsdPhij(b,Bt,Bs,j);
                
                hess(j+2,1)   = sum((dydBs.*dydPj + (y-m).*dy2dBsdPj) ./ h.^2);
                hess(1,j+2)   = hess(j+2,1);
                hess(j+2,2)   = sum((dydBt.*dydPj + (y-m).*dy2dBtdPj) ./ h.^2);
                hess(2,j+2)   = hess(j+2,2);
                hess(j+2,j+2) = sum(dydPj.^2 ./ h.^2);
                
                for k = (j+1):numel(c)
                    dydPk = dMdPhij(b,Bt,Bs,k);
                    hess(j+2,k+2) = sum(dydPj.*dydPk ./ h.^2);
                    hess(k+2,j+2) = hess(j+2,k+2);
                end
            end
            
            %% Armijo Update
            fold = func;
            
            dir = hess \ grad;
            Bs = Bs - dir(1);
            Bt = Bt - dir(2);
            for j = 1:numel(c)
                c(j) = c(j) - dir(j+2);
            end
            
            y    = M(b,Bt,Bs,c);
            fnew = 0.5 * sum((y-m).^2 ./ h.^2);
            k    = 1;
            while fnew >= fold && k <= 52
                dir = dir / 2;
                Bs = Bs + dir(1);
                Bt = Bt + dir(2);
                for j = 1:numel(c)
                    c(j) = c(j) + dir(j+2);
                end
                
                y    = M(b,Bt,Bs,c);
                fnew = 0.5 * sum((y-m).^2 ./ h.^2);
                k    = k + 1;
            end
            
            if k == 53
                dir = hess(1:2,1:2) \ grad(1:2);  
                
                Bs = Bs - dir(1);
                Bt = Bt - dir(2);

                y    = M(b,Bt,Bs,c);
                fnew = 0.5 * sum((y-m).^2 ./ h.^2);
                k    = 1;
                while fnew >= fold && k <= 52
                    dir = dir / 2;
                    Bs = Bs + dir(1);
                    Bt = Bt + dir(2);

                    y    = M(b,Bt,Bs,c);
                    fnew = 0.5 * sum((y-m).^2 ./ h.^2);
                    k    = k + 1;
                end
                
                maxdir = max(abs(dir./[Bs;Bt]));
            else
                maxdir = max(abs(dir./[Bs;Bt;c]));
            end
        end
        
        if i == 1
            %% Estimate Optimal Basis Function Coefficients
            y = Phi(b,Bt,Bs);

            %lsqmat = [y,y.^2,y.^3,y.^4,y.^5];
            lsqmat = [y,y.^2,y.^3];
            weight = diag(1./h.^2);
            c = (lsqmat.'*weight*lsqmat) \ (lsqmat.' * weight * m);
        end
    end
    
    x = linspace(0,max([2,Bs,max(b)]));
    y = M(x,Bt,Bs,c);
    dy = dMdB(x,Bt,Bs,c);
    
%     figure(1);clf;
%     plot(x,y);hold on;
%     scatter(b,m);
%     title('M-B Curve');
%     
%     figure(2);clf;
%     plot(x,dy);
%     title('dMdB-B Curve');
%     
%     figure(3);clf;
%     plot(x/mu_o-y,x);hold on;
%     scatter(h,b);
%     title('B-H Curve');
%     
%     figure(4);clf;
%     plot(x,1./(1-mu_o*dy));
%     title('Incremental Relative Permeability');
%     
%     figure(5);clf;
%     plot(x,x./(x/mu_o-y)/mu_o);
%     title('Effective Relative Permeability');
end

%% Symbolic Function and Derivatives
% syms Bs Bt x;
% M = 1/(1+exp((x-Bs)/Bt));
% M = int(M,x);
% M = simplify(M-subs(M,x,0));
% M = simplify(M);
% 
% dMdBt = simplify(diff(M,Bt));
% dMdBs = simplify(diff(M,Bs));
% 
% dM2dBt2   = simplify(diff(dMdBt,Bt));
% dM2dBs2   = simplify(diff(dMdBs,Bs));
% dM2dBtdBs = simplify(diff(dMdBt,Bs));

%% Magnetization Curve Evalution
function y = M(x,Bt,Bs,c)
    z = Phi(x,Bt,Bs);
    y = 0*z;
    for i = 1:numel(c)
        y = y + c(i) * z.^i;
    end
end

function y = dMdB(x,Bt,Bs,c)
    z0 = Phi(x,Bt,Bs);
    z1 = dPhidx(x,Bt,Bs);
    y = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * i * z0.^(i-1) .* z1;
    end
end

function y = dMdBs(x,Bt,Bs,c)
    z0 = Phi(x,Bt,Bs);
    z1 = dPhidBs(x,Bt,Bs);
    y = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * i * z0.^(i-1) .* z1;
    end
end

function y = dMdBt(x,Bt,Bs,c)
    z0 = Phi(x,Bt,Bs);
    z1 = dPhidBt(x,Bt,Bs);
    y = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * i * z0.^(i-1) .* z1;
    end
end

function y = dMdPhij(x,Bt,Bs,j)
    y = Phi(x,Bt,Bs).^j;
end

function y = dM2dBs2(x,Bt,Bs,c)
    z0 = Phi(x,Bt,Bs);
    z1 = dPhidBs(x,Bt,Bs);
    z2 = dPhi2dBs2(x,Bt,Bs);
    y  = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * (i * z0.^(i-1) .* z2);
        if i > 1
            y = y + c(i) * (i * (i-1) * z0.^(i-2) .* z1.^2);
        end
    end
end

function y = dM2dBt2(x,Bt,Bs,c)
    z0 = Phi(x,Bt,Bs);
    z1 = dPhidBt(x,Bt,Bs);
    z2 = dPhi2dBt2(x,Bt,Bs);
    y  = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * (i * z0.^(i-1) .* z2);
        if i > 1
            y = y + c(i) * i * (i-1) * z0.^(i-2) .* z1.^2;
        end
    end
end

function y = dM2dBtdBs(x,Bt,Bs,c)
    z0  = Phi(x,Bt,Bs);
    zt  = dPhidBt(x,Bt,Bs);
    zs  = dPhidBs(x,Bt,Bs);
    zts = dPhi2dBtdBs(x,Bt,Bs);
    y   = 0*z0;
    for i = 1:numel(c)
        y = y + c(i) * i * z0.^(i-1) .* zts;
        if i > 1
            y = y + c(i) * i * (i-1) * z0.^(i-1) .* zt .* zs;
        end
    end
end

function y = dM2dBtdPhij(x,Bt,Bs,j)
    y = j*Phi(x,Bt,Bs).^(j-1).*dPhidBt(x,Bt,Bs);
end

function y = dM2dBsdPhij(x,Bt,Bs,j)
    y = j*Phi(x,Bt,Bs).^(j-1).*dPhidBs(x,Bt,Bs);
end

%% Basis Function and Derivatives
function y = Phi(x,Bt,Bs)
    y = Bt*(log(exp(-Bs/Bt) + 1) - log(exp(-Bs/Bt)+exp(-x/Bt)));
end

function y = dPhidx(x,Bt,Bs)
    y = exp(-x/Bt)./(exp(-Bs/Bt)+exp(-x/Bt));
end

function y = dPhidBs(x,Bt,Bs)
    y = 1./(exp((Bs - x)./Bt) + 1) - 1./(exp(Bs./Bt) + 1);
end

function y = dPhidBt(x,Bt,Bs)
    y = log(1./exp(Bs./Bt) + 1) - log(1./exp((Bs - x)./Bt) + 1) + Bs./(Bt.*exp(Bs./Bt).*(1./exp(Bs./Bt) + 1)) - (Bs - x)./(Bt.*exp((Bs - x)./Bt).*(1./exp((Bs - x)./Bt) + 1));
end

function y = dPhi2dBs2(x,Bt,Bs)
    y = exp(Bs./Bt)./(Bt.*(exp(Bs./Bt) + 1).^2) - exp((Bs - x)./Bt)./(Bt.*(exp((Bs - x)./Bt) + 1).^2);
end

function y = dPhi2dBt2(x,Bt,Bs)
    y = -(exp((Bs - x)./Bt).*(Bs.^2 - 2.*Bs.*x + Bs.^2.*exp((2.*Bs)./Bt) + x.^2 + x.^2.*exp((2.*Bs)./Bt) - 2.*Bs.*x.*exp((2.*Bs)./Bt)) - exp(Bs./Bt).*(Bs.^2 + Bs.^2.*exp((2.*(Bs - x))./Bt) + exp((Bs - x)./Bt).*(- 2.*x.^2 + 4.*Bs.*x)))./(Bt.^3.*(exp(Bs./Bt) + 1).^2.*(exp((Bs - x)./Bt) + 1).^2);
end

function y = dPhi2dBtdBs(x,Bt,Bs)
    y = (exp((Bs - x)./Bt).*(Bs - x + Bs.*exp((2.*Bs)./Bt) - x.*exp((2.*Bs)./Bt)) - exp(Bs./Bt).*(Bs + Bs.*exp((2.*(Bs - x))./Bt) + 2.*x.*exp((Bs - x)./Bt)))./(Bt.^2.*(exp(Bs./Bt) + 1).^2.*(exp((Bs - x)./Bt) + 1).^2);
end