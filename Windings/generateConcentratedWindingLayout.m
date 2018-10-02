function W = generateConcentratedWindingLayout(Np,Nt,Nl)
    % Algorithm verified against multiple layouts given at https://www.emetor.com/edit/windings/
    % #TODO, Generalize to arbitrary poly-phase windings
    
    assert(Np*3 >= Nt);
    assert(Nt >= Np)
    assert(Nl == 1 || Nl == 2);
    if Nl == 1
        assert(mod(Nt,2) == 0);
        assert(mod(Nt,3) == 0);
    end
    
    if Nl == 1
        r = gcd(Np,Nt/2);
    else
        r = gcd(Np,Nt);
    end
    
    if mod(Np / r, 2) == 1
        antiperiodic = true;
        r = r / 2;
    else
        antiperiodic = false;
    end
    Np = Np / r;
    Nt = Nt / r;
    
    a_tooth = linspace(0,2*pi,7);
    a_tooth(end) = [];
    B_tooth = exp(1i*a_tooth);
    
    a_ideal = linspace(0,2*pi,Nt*r+1);
    a_ideal(end) = [];
    B_ideal = exp(1i*a_ideal*Np*r/2);
    
    B_sample = zeros(1,Nt*r);
    W        = zeros(1,Nt);
    d        = zeros(1,6);
    for i = 1:(Nt/(1+antiperiodic))
        if i ~= 1
            for j = 1:6
                if antiperiodic
                    I = (i:Nt:floor(Nt*r/2));
                    B_sample(I) = B_tooth(j);
                    
                    k = mod(j+2,6)+1;
                    I = I + Nt / 2;
                    
                    B_sample(I) = B_tooth(k);
                else
                    B_sample(i:Nt:end) = B_tooth(j);
                end
                
                d(j) = norm(B_sample-B_ideal);
            end
            
            [d_min,~] = min(d);
            I = find(abs(d-d_min) < sqrt(eps) * d_min);
            k = zeros(length(I),1);
            for j = 1:length(I)
                k(j) = sum(W == I(j));
            end
            [~,k] = min(k);
            
            W(i) = I(k);
        else
            W(i) = 1;
        end
        B_sample(i:Nt:end) = B_tooth(W(i));
    end
    
    if antiperiodic
        W((end/2+1):end) = mod(W(1:(end/2))+2,6)+1;
    end
        
    if Nl == 1
        W = W(1:2:end);
    end
    
    wdg  = zeros(numel(W),2);
    I    = (W > 3);
    W(I) = 3 - W(I);
    I    = (mod(W,2) == 0);
    W(I) = -W(I);
    
    for i = 1:numel(W);
        wdg(i,1) = W(i);
        wdg(i,2) = -W(i);
    end
    
    while any(abs(wdg(end,:)) == 1)
        wdg = circshift(wdg,1);
    end
    
    if wdg(1,1) == -1
        wdg = -wdg;
    end
    
    if Nl == 2
        wdg(:,2) = circshift(wdg(:,2),-1);
    end
    
    W = wdg;
end