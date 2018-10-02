function X = idscfft(Y,N,DIM)
    if nargin < 3
        if isrow(Y)
            DIM = 2;
        else
            DIM = 1;
        end
    end
    
    Ma = size(Y);
    Ma = Ma(DIM);
    if mod(Ma,2) == 0
        Mb = Ma - 1;
    else
        Mb = Ma;
    end
    
    if nargin < 2 || isempty(N)
        N = Ma;
    end
    
    subs      = repmat({':'},1,ndims(Y));
    
    subs{DIM} = 2:2:Mb;
    X         = Y(subs{:});
    
    subs{DIM} = 3:2:Mb;
    X         = X + 1i * Y(subs{:});
    
    if mod(Ma,2) == 0
        subs{DIM} = Ma;
        X         = cat(DIM, cat(DIM, X, Y(subs{:})), flipdim(conj(X), DIM));
    else
        X         = cat(DIM, X, flipdim(conj(X), DIM));
    end
    
    subs{DIM} = 1;
    X         = cat(DIM, Y(subs{:}), X);
    X         = ifft(X,N,DIM,'Symmetric') * N;
end