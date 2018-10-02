function [Y,Z] = dscfft(X,N,DIM)
    if nargin < 3
        if isrow(X)
            DIM = 2;
        else
            DIM = 1;
        end
    end
    
    if nargin < 2 || isempty(N)
        N = size(X);
        N = N(DIM);
    end
    
    a = fft(X,N,DIM);
    
    a = a / N;
    
    subs = repmat({':'},1,ndims(a));
    
    
    if mod(N,2) == 0
        Ma = N / 2;
        Mb = N / 2;
        Mc = N - 1;
    else
        Ma = (N + 1) / 2;
        Mb = (N + 1) / 2;
        Mc = N;
    end
    
    subs{DIM}  = 1:Ma;
    a          = a(subs{:});
    
    subs{DIM}  = 2:Ma;
    a(subs{:}) = a(subs{:});
    
    if nargout == 1 || nargout == 0    
        subs{DIM}  = 2:Mb;
        b          = a(subs{:});
        
        subs{DIM}  = 1:Mc;
        Y          = X(subs{:});
        
        subs{DIM}  = [1,2:2:Mc];
        Y(subs{:}) = real(a);

        subs{DIM}  = 3:2:Mc;
        Y(subs{:}) = imag(b);
    elseif nargout == 2
        Y = real(a);
        Z = imag(a);
    end
end