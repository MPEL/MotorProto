function [Rk,Rs,Sk] = makeRegionAndStrandSets(Pk,Ps)
    Nb = numel(Pk);
    
    Rk = cell(1,Nb);
    Rs = cell(1,Nb);
    Sk = cell(1,Nb);
    for i = 1:Nb
        Ns = numel(Pk{i});
        
        Pkcat = vertcat(Pk{i}{:});
        Pscat = vertcat(Ps{i}{:});

        [Pkcat,J] = sortrows(Pkcat);
        Pscat     = Pscat(J,:);

        Pk{i}     = Pk{i}(J);
        Ps{i}     = Ps{i}(J);

        [Pkcat,I] = unique(Pkcat,'rows','first');
        Pscat     = Pscat(I,:);

        Rk{i} = cell(1,numel(I));
        Rs{i} = cell(1,numel(I));
        Sk{i} = cell(1,numel(I));

        %% Construct Region Sets
        %First get unique region indices
        for m = 1:size(Pkcat,1)
            [Rk{i}{m},K] = unique(Pkcat(m,:));
            Rs{i}{m}     = Pscat(m,K);
            if m < numel(I)
                Sk{i}{m} = I(m):(I(m+1)-1);
            else
                Sk{i}{m} = I(m):Ns;
            end
        end
        
        %If any two sets share an index, merge them
        j = 1;
        N = length(Rk{i});
        while j < N
            k = j + 1;
            while k <= N
                J = intersect(Rk{i}{j},Rk{i}{k});
                if numel(J) > 0
                    K        = [Rk{i}{j},Rk{i}{k}];
                    S        = [Rs{i}{j},Rs{i}{k}];
                    
                    [~,I]    = unique(K);
                    
                    Rk{i}{j} = K(I);
                    Rs{i}{j} = S(I);
                    Sk{i}{j} = [Sk{i}{j},Sk{i}{k}];
                    
                    Rk{i}(k) = [];
                    Rs{i}(k) = [];
                    Sk{i}(k) = [];
                    
                    N        = N - 1;
                else
                    k = k + 1;
                end
            end
            j = j + 1;
        end
    end
end