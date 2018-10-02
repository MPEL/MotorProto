classdef Circuit < Component
    properties
        CouplingType = CouplingTypes.Dynamic;
        Linear       = true;
        
        %% Current Constraint Coupling Properties
        PathSets       = {};
        PathPolarity   = {};        
        RegionSets     = {};
        RegionPolarity = {};
        StrandSets     = {};
        
        %% Voltage Constraint Coupling Properties
        TurnSets     = {};
        TurnPolarity = {};
        TurnFactor   = {};
        
        ParallelPaths = 1;
        
        Index %Set by MatrixFactory after getSize allocation request
        
        %% Matrix Struct
        Matrices
    end
    
    properties (Dependent, SetAccess = private)
        Nb
        Nr
        
      	StrandCurrentIndex
        ScalarPotentialIndex
    end
    
    properties (Abstract, Dependent, SetAccess = private)
        BundleVoltageIndex
        BundleCurrentIndex
    end
    
    methods
        function this = Circuit(varargin)
            this = this@Component(varargin{:});
        end
        
    	function N = getSize(this) %Matrix factory allocation request
            N = 0;
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    R = this.RegionSets;
                    for i = 1:numel(R)
                        for m = 1:numel(R{i})
                            N = N + numel(R{i}{m}); %E_{phi,k} for each strand cross section
                        end
                      	N = N + numel(R{i});        %i_{i,m} for each unique strand current
                    end
                otherwise
            end
        end
        
        %% Getters
        function value = get.Nb(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    value = numel(this.RegionSets);
                case CouplingTypes.Static
                    value = numel(this.TurnSets);
                otherwise
            end
        end
        
        function value = get.Nr(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    value = 0;
                    Rk = this.RegionSets;
                    for i = 1:numel(Rk);
                        for j = 1:numel(Rk{i})
                            value = value + numel(Rk{i}{j});
                        end
                    end
                case CouplingTypes.Static
                    value = 0;
                    Tk = this.TurnSets;
                    for i = 1:numel(Tk)
                        value = value + numel(Tk{i});
                    end
                otherwise
            end
        end
        
        function value = get.ParallelPaths(this)
            Npp = this.ParallelPaths;
            if (numel(Npp)) == 1 && (this.Nb > 1)
                value = Npp * ones(1, this.Nb);
            else
                assert(numel(Npp) == this.Nb);
                value = Npp;
            end 
        end
        
       	function value = get.ScalarPotentialIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    R = this.RegionSets;
                    
                    value = cell(1, numel(R));
                    index = this.Index;
                    
                    m = 1;
                    for i = 1:numel(R)
                        value{i} = cell(1,numel(R{i}));
                        for j = 1:numel(R{i})
                            n = m + numel(R{i}{j}) - 1;
                            value{i}{j} = index(m:n);
                            m = n + 1;
                        end
                    end
                otherwise
                    value = [];
            end
        end
        
        function value = get.StrandCurrentIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    R = this.RegionSets;
                    
                    value = cell(1, numel(R));
                    index = this.Index;
                    
                    m = 1;
                    for i = 1:numel(R)
                        for j = 1:numel(R{i})
                            m = m + numel(R{i}{j});
                        end
                    end
                    
                    for i = 1:numel(R)
                        n = m + numel(R{i}) - 1;
                        value{i} = index(m:n);
                        m = n + 1;
                    end
                otherwise
                    value = [];
            end
        end
        
        %% Setters
        function this = set.CouplingType(this, value)
            if ischar(value)
                this.CouplingType = CouplingTypes.(value);
            else
                this.CouplingType = CouplingTypes(value);
            end
        end
        
        %% Size Functions
     	function varargout = getBundleSizes(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    Rk = this.RegionSets;
                    Sk = this.StrandSets;
                    
                    Nb  = numel(Rk);   %Number of bundles
                    Nib = zeros(1,Nb); %Number of unique currents per bundle
                    Nri = cell(1,Nb);  %Number of regions associated with each unique current
                    Nss = cell(1,Nb);  %Size of the strand set

                    for i = 1:Nb
                        Nib(i) = numel(Rk{i});
                        Nri{i} = zeros(1,Nib(i));
                        Nss{i} = zeros(1,Nib(i));
                        for j = 1:Nib(i)
                            Nri{i}(j) = numel(Rk{i}{j});
                            Nss{i}(j) = numel(Sk{i}{j});
                        end
                    end

                    varargout = {Nb,Nib,Nri,Nss};
                case CouplingTypes.Static
                    Tk = this.TurnSets;
                    
                    Nb  = numel(Tk);
                    Nki = zeros(1,Nb);
                    for i = 1:Nb
                        Nki(i) = numel(Tk{i});
                    end
                    
                    varargout = {Nb, Nki};
                otherwise
            end
        end
        
        %% Build Function
    	function this = build(this, coupling, bcFun)
            matrices = struct('Kfc', [], 'Kcf', [], 'Kcc', [],...
                              'Cfc', [], 'Ccf', [], 'Ccc', []);
                          
            matrices.Kfc = this.Kfc(coupling);
            matrices.Kcf = this.Kcf(coupling);
            matrices.Kcc = this.Kcc(coupling);
            matrices.Cfc = this.Cfc(coupling);
            matrices.Ccf = this.Ccf(coupling);
            matrices.Ccc = this.Ccc(coupling);
            
            matrices.Kfc = bcFun(matrices.Kfc, 'both');
            matrices.Kcf = bcFun(matrices.Kcf, 'both');
            matrices.Kcc = bcFun(matrices.Kcc, 'both');
            matrices.Cfc = bcFun(matrices.Cfc, 'both');
            matrices.Ccf = bcFun(matrices.Ccf, 'both');
            matrices.Ccc = bcFun(matrices.Ccc, 'both');
            
            this.Matrices = matrices;
        end
        
        %% Matrix Functions
        function k = K(this, ~, h)
            k = this.Matrices.Kfc + this.Matrices.Kcf * h + this.Matrices.Kcc * h;
        end
        
        function c = C(this, ~, h, s)
            c = this.Matrices.Cfc / h + this.Matrices.Ccf * s / h + this.Matrices.Ccc * s /h;
        end
        
        %% Matrices
        function kfc = Kfc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    ind = this.ScalarPotentialIndex;
                case CouplingTypes.Static
                    ind = this.BundleCurrentIndex;
                otherwise
            end
            
            if isempty(ind)
                kfc = sparse(coupling.Unknowns, coupling.Unknowns);
            else
                kfc = this.Circuit2FEA(coupling, ind, coupling.Unknowns, coupling.Unknowns);
            end
        end
        
        function kcf = Kcf(~, coupling)
            kcf = sparse(coupling.Unknowns, coupling.Unknowns);
        end
        
       	function ccf = Ccf(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    ind = this.ScalarPotentialIndex;
                case CouplingTypes.Static
                    ind = this.BundleCurrentIndex;
                otherwise
            end
            
            if isempty(ind)
                ccf = sparse(coupling.Unknowns, coupling.Unknowns);
            else
                ccf = this.Circuit2FEA(coupling, ind, coupling.Unknowns, coupling.Unknowns);
                ccf = ccf.';
            end
        end

        function cfc = Cfc(~, coupling)
            cfc = sparse(coupling.Unknowns, coupling.Unknowns);
        end
        
        %% Post Processing
        % Current
        function x2i = X2I(this, coupling)
            ind = this.BundleCurrentIndex;
            
            if isempty(ind)
                x2i = sparse(this.Nb, coupling.Unknowns);
            else
                Nb  = this.Nb;
                x2i = sparse(1:Nb, ind, ones(1, Nb), Nb, coupling.Unknowns);
            end
        end

        function x_t2i = X_t2I(this, coupling)
            x_t2i = sparse(this.Nb, coupling.Unknowns);
        end
        
        % Voltage
        function x2v = X2V(this, coupling)
            ind = this.BundleVoltageIndex;
            
            if isempty(ind)
                x2v = sparse(this.Nb, coupling.Unknowns);
            else
                Nb  = this.Nb;
                rows = 1:Nb;
                cols = ind;
                vals = ones(1, Nb);
                x2v  = sparse(rows, cols, vals, Nb, coupling.Unknowns);
            end
        end
        
     	function x_t2v = X_t2V(this, coupling)
            ind = this.BundleVoltageIndex;
            
            if isempty(ind)
                x_t2v = this.A2Lambda(coupling);
            else
                x_t2v = sparse(this.Nb, coupling.Unknowns);
            end
        end
        
        % Flux Linkage
        function f2l = F2Lambda(this, ~)
            %NOOP, Flux linkage is determined from the magnetic vector potential
            f2l = sparse(this.Nb, this.Nb);
        end
        
        function x2l = X2Lambda(this, coupling)
            x2l = this.A2Lambda(coupling);
        end

        function x_t2l = X_t2Lambda(this, coupling)
            %NOOP, Flux linkage is determined from the magnetic vector potential
         	x_t2l = sparse(this.Nb, coupling.Unknowns);
        end
        
        % Electric Field
    	function x2e = X2E(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %Part of E due to grad(Phi)
                    coef = ones(size(coupling.Conductivity));
                    ind  = this.ScalarPotentialIndex;
                case CouplingTypes.Static
                    %Part of E due to bundle currents
                    coef = 1./coupling.Conductivity;
                    ind  = this.BundleCurrentIndex;
                otherwise
            end
            
            if isempty(ind)
                x2e = sparse(coupling.Elements, coupling.Unknowns);
            else
                x2e = this.Circuit2Field(coupling, coef, ind, coupling.Elements, coupling.Unknowns);
            end
        end
        
    	function x_t2e = X_t2E(this, coupling)
            %Part of E from dA/dt
            coef  = ones(size(coupling.Conductivity));
            x_t2e = this.A2Field(coupling, coef, coupling.Elements, coupling.Unknowns);
        end
        
        % Current Density
    	function x2j = X2J(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %Part of J due to sigma*grad(Phi)
                    ind  = this.ScalarPotentialIndex;
                    coef = coupling.Conductivity;
                case CouplingTypes.Static
                    %Part of J due to bundle currents
                    ind  = this.BundleCurrentIndex;
                    coef = ones(size(coupling.Conductivity));
                otherwise
            end
            
            if isempty(ind)
                x2j = sparse(coupling.Elements, coupling.Unknowns);
            else
                x2j = this.Circuit2Field(coupling, coef, ind, coupling.Elements, coupling.Unknowns);
            end
        end
        
        function x_t2j = X_t2J(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %Part of J from sigma * dA/dt
                    x_t2j = this.A2Field(coupling, coupling.Conductivity, coupling.Elements, coupling.Unknowns);
                case CouplingTypes.Static
                    %NOOP, Current density is assumed to be uniform
                    m     = coupling.Elements;
                    n     = coupling.Unknowns;
                    x_t2j = {sparse(m, n), sparse(m, n), sparse(m, n)};
                otherwise
            end
        end
        
        %% Matrix Primitives
        function c2f = Circuit2FEA(this, coupling, index, nRows, nCols)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    [Nb, Ns, Nr, ~] = this.getBundleSizes;
                    Rk = this.RegionSets;
                    
                    sigma    = coupling.Conductivity;
                    integral = coupling.Integral;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);

                    %% One column for each E_{phi,k} coupling to FEA equations
                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Ns(i)
                            for k = 1:Nr{i}(j)
                                m   = Rk{i}{j}(k);
                                pos = pos + 1;

                                [col, row, val] = find(integral{m});
                                col = index{i}{j}(k) * col;
                                val = sigma(m) * val;
                                
                                rows{pos} = row;
                                cols{pos} = col;
                                vals{pos} = val;
                            end
                        end
                    end
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    c2f = sparse(rows, cols, vals, nRows, nCols);
                case CouplingTypes.Static
                    [Nb, Nk] = this.getBundleSizes;
                    Npp = this.ParallelPaths;
                    
                    Tk = this.TurnSets;
                    Ts = this.TurnPolarity;
                    Tf = this.TurnFactor;
                    
                    area = coupling.Area;
                    integral = coupling.Integral;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);
                    
                    pos = 0;
                    for i = 1:Nb %Bundle current to current density transformation
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            c = -Ts{i}(j) * Tf{i}(j) / (area(k) * Npp(i));
                            
                            [col, row, val] = find(integral{k});
                            col = index(i) * col;
                            val = c * val;
                            
                            pos = pos + 1;
                            rows{pos} = row;
                            cols{pos} = col;
                            vals{pos} = val;
                        end
                    end
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    c2f = sparse(rows, cols, vals, nRows, nCols);
                otherwise
            end
        end
        
        %% Post Processing Primitives
        function a2c = A2Lambda(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    [Nb, Nk, ~, ~] = this.getBundleSizes;
                    
                    Rk = this.RegionSets;
                    Pk = this.PathSets;
                    Ps = this.PathPolarity;
                    
                    Npp = this.ParallelPaths;
                    
                    ls       = coupling.Length;
                    integral = coupling.Integral;
                    area     = coupling.Area;
                    
                    %% Calculate flux linkage contribution from each region
                    Lk = cell(1,0);
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            for k = Rk{i}{j}
                                Lk{k} = ls * integral{k} / (area(k) * Npp(i));
                            end
                        end
                    end
                    
                    %% Calculate flux linkage for each strand
                    Ls = cell(1,Nb);
                    for i = 1:Nb
                        Ns    = numel(Pk{i});
                        Ls{i} = cell(1, Ns);
                        for j = 1:Ns
                            Ls{i}{j} = Ps{i}{j}(1) * Lk{Pk{i}{j}(1)};
                            for k = 2:numel(Pk{i}{j})
                                Ls{i}{j} = Ls{i}{j} + Ps{i}{j}(k) * Lk{Pk{i}{j}(k)};
                            end
                        end
                    end
                    
                    %% Average flux linkage across all strands for each bundle
                    Lb = cell(Nb, 1);
                    for i = 1:Nb
                        Ns    = numel(Ls{i});
                        Lb{i} = Ls{i}{1};
                        for j = 2:Ns
                            Lb{i} = Lb{i} + Ls{i}{j};
                        end
                        Lb{i} = Lb{i} / Ns;
                    end
                    
                    a2c = cell2mat(Lb);
                case CouplingTypes.Static
                    [Nb, Nk] = this.getBundleSizes;
                    
                    Tk = this.TurnSets;
                    Ts = this.TurnPolarity;
                    Tf = this.TurnFactor;
                    
                    Npp = this.ParallelPaths;
                    
                    ls       = coupling.Length;
                    fp       = coupling.ModeledFraction;
                    area     = coupling.Area;
                    integral = coupling.Integral;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);
                    
                    pos  = 0;
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            c = Ts{i}(j) * Tf{i}(j) * ls / (fp * area(k) * Npp(i));
                            
                            [row,col,val] = find(integral{k});
                            row = i * row;
                            val = c * val;
                            
                            pos = pos + 1;
                            rows{pos} = row;
                            cols{pos} = col;
                            vals{pos} = val;
                        end
                    end
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    a2c = sparse(rows, cols, vals, Nb, coupling.Unknowns);
                otherwise
            end
        end
    
        function c2f = Circuit2Field(this, coupling, coef, index, nRows, nCols)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %Part of E due to grad(Phi)
                    [Nb, Ns, Nr, ~] = this.getBundleSizes;
                    Rk = this.RegionSets;

                    r2el = coupling.Region2Element;

                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);

                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Ns(i)
                            for k = 1:Nr{i}(j)
                                m   = Rk{i}{j}(k);
                                pos = pos + 1;
                                
                                [row, col, val] = find(r2el{m});
                                col = index{i}{j}(k) * col;
                                val = -coef(m) * val;
                                
                                rows{pos} = row.';
                                cols{pos} = col.';
                                vals{pos} = val.';
                            end
                        end
                    end

                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];

                    c2f  = sparse(rows, cols, vals, nRows, nCols);
                case CouplingTypes.Static
                    %Part of E due to bundle currents
                    [Nb, Nk] = this.getBundleSizes;

                    Tk = this.TurnSets;
                    Ts = this.TurnPolarity;
                    Tf = this.TurnFactor;
                    
                    Npp = this.ParallelPaths;
                    
                    area = coupling.Area;
                    r2el = coupling.Region2Element;

                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);

                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            c = coef(k) * Ts{i}(j) * Tf{i}(j) / (area(k) * Npp(i));

                            [row, col, val] = find(r2el{k});
                            col = index(i) * col;
                            val = c * val;

                            pos = pos + 1;
                            rows{pos} = row.';
                            cols{pos} = col.';
                            vals{pos} = val.';
                        end
                    end
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];

                    c2f = sparse(rows, cols, vals, nRows, nCols);
                otherwise
            end
        end
        
        function a2f = A2Field(this, coupling, coef, nRows, nCols)
            switch this.CouplingType
                case CouplingTypes.Dynamic                    
                    [Nb, Nk, ~, ~] = this.getBundleSizes;
                    Rk = this.RegionSets;
                    
                    rows = cell(3,0);
                    cols = cell(3,0);
                    vals = cell(3,0);
                    
                    n2el = coupling.Node2Element; 
                    
                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            for k = Rk{i}{j}
                                for l = 1:3
                                    [ra, ca, va] = find(n2el{l,k});
                                    va = -coef(k) * va;

                                    pos = pos + 1;
                                    rows{l, pos} = ra.';
                                    cols{l, pos} = ca.';
                                    vals{l, pos} = va.';
                                end
                            end
                        end
                    end
                    
                    a2f = cell(1,3);
                    for i = 1:3
                        a2f{i} = sparse([rows{i,:}], [cols{i,:}], [vals{i,:}], nRows, nCols);
                    end
                case CouplingTypes.Static
                    [Nb, Nk] = this.getBundleSizes;
                    Tk = this.TurnSets;
                    
                    integral = coupling.Integral;
                    area     = coupling.Area;
                    r2el     = coupling.Region2Element;
                    n2el     = coupling.Node2Element;
                    
                    rows = cell(3,0);
                    cols = cell(3,0);
                    vals = cell(3,0);
                    
                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            
                            [ra, ~, ~]  = find(r2el{k});
                            [~, ca, va] = find(integral{k});
                            va = coef(k) * va / area(k);
                            
                            m = numel(ca);
                            n = numel(ra);
                            for p = 1:n %Part of E from E_{phi} due to average of dA/dt
                                pos = pos + 1;
                                for l = 1:3
                                    rows{l,pos} = ra(p) * ones(1, m);
                                    cols{l,pos} = ca;
                                    vals{l,pos} = va;
                                end
                            end
                            
                            pos = pos + 1;
                            for l = 1:3 %Part of E from dA/dt
                             	[row, col, val] = find(n2el{l,k});
                                val = -coef(k) * val;
                                
                                rows{l,pos} = row.';
                                cols{l,pos} = col.';
                                vals{l,pos} = val.';
                            end
                        end
                    end
                    
                    a2f = cell(1,3);
                    for i = 1:3
                        a2f{i} = sparse([rows{i,:}], [cols{i,:}], [vals{i,:}], nRows, nCols);
                    end
                otherwise
            end
        end
    end
end