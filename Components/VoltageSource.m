classdef VoltageSource < Source
    properties (Constant)
        Type = SourceTypes.VoltageSource
    end
    
    properties (Dependent, SetAccess = private)
        BundleVoltageIndex
        BundleCurrentIndex
        CommonModeVoltageIndex
    end
    
    methods
        function this = VoltageSource(varargin)
            this = this@Source(varargin{:});
        end
        
        function N = getSize(this) %Matrix factory allocation request
            N = getSize@Source(this); %N for strand voltages and currents in dynamic case
            N = N + this.Nb;          %i_i for each bundle current
            
            if this.CouplingType == CouplingTypes.Dynamic
                N = N + this.Nb;      %v_i for each bundle voltage
            end
            
            N = N + 1;                %v_o, common mode voltage
        end
        
        %% Indexing properties
        function value = get.BundleVoltageIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    R = this.RegionSets;
                    
                    m = 1;
                    for i = 1:numel(R)
                        m = m + numel(R{i});
                        for j = 1:numel(R{i});
                            m = m + numel(R{i}{j});
                        end
                    end
                    n = m + numel(R) - 1;
                    
                    value = this.Index(m:n);
                otherwise
                    value = [];
            end
        end
        
        function value = get.BundleCurrentIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    R = this.RegionSets;
                    
                    m = 1;
                    for i = 1:numel(R)
                        m = m + numel(R{i});
                        for j = 1:numel(R{i});
                            m = m + numel(R{i}{j});
                        end
                    end
                    m = m + numel(R);
                    n = m + numel(R) - 1;
                    
                    value = this.Index(m:n);
                otherwise
                    Nb = this.Nb;
                    value = this.Index(1:Nb);
            end
        end
        
        function value = get.CommonModeVoltageIndex(this)
            value = this.Index(end);
        end
        
        %% Matrices
        function kcc = Kcc(this, coupling)
            % #TODO - Add external circuit elements
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    Rk = this.RegionSets;
                    Rs = this.RegionPolarity;
                    
                    [Nb, Nib, Nri, Nss] = this.getBundleSizes;
                    Npp = this.ParallelPaths;
                    
                    sigma = coupling.Conductivity;
                    area  = coupling.Area;
                    fp    = coupling.ModeledFraction;
                    ls    = coupling.Length;
                    
                    K = this.ScalarPotentialIndex;
                    J = this.StrandCurrentIndex;
                    I = this.BundleVoltageIndex;
                    H = this.BundleCurrentIndex;
                    G = this.CommonModeVoltageIndex;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);

                    %% One row for each E_{phi,k} for current constraints
                    pos = 0;
                    for i = 1:Nb
                        for j = 1:Nib(i)
                            for k = 1:Nri{i}(j)
                                m = Rk{i}{j}(k);
                                s = Rs{i}{j}(k);
                                
                                pos = pos + 1;
                                rows{pos} = [K{i}{j}(k), K{i}{j}(k)];
                                cols{pos} = [K{i}{j}(k), J{i}(j)];
                                vals{pos} = [sigma(m) * area(m), s];
                            end
                        end
                    end

                    %% One row for each strand current i_{i,m}, #TODO, Add external turn impedence
                    for i = 1:Nb
                        for j = 1:Nib(i)
                            s = Rs{i}{j};
                            
                            pos = pos + 1;
                            rows{pos} = J{i}(j) * ones(1, Nri{i}(j) + 2);
                            cols{pos} = [K{i}{j}, J{i}(j), I(i)];
                            vals{pos} = [s, 0 * fp/ls * Nss{i}(j), fp/ls * Nss{i}(j) * Npp(i)];
                        end
                    end

                    %% One row for each bundle voltage v_{i}
                    for i = 1:Nb
                        pos = pos + 1;
                        rows{pos} = I(i) * ones(1, Nib(i) + 1);
                        cols{pos} = [J{i}, H(i)];
                        vals{pos} = fp/ls * [Nss{i} * Npp(i), -1];
                    end

                    %% One row for each bundle current i_{i}, #TODO, Add source impedence
                    for i = 1:Nb
                        pos = pos + 1;
                        rows{pos} = H(i) * ones(1, 3);
                        cols{pos} = [I(i), H(i), G];
                        vals{pos} = [-fp/ls, 0 * fp/ls, -fp/ls];
                    end
                    
                    %% One row for common mode current constraint
                    pos = pos + 1;
                    rows{pos} = G * ones(1, Nb);
                    cols{pos} = H;
                    vals{pos} = -fp/ls * ones(1, Nb);
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    kcc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    % #TODO - Add external circuit elements
                    [Nb, Nk] = this.getBundleSizes;
                    
                    Tk = this.TurnSets;
                    Tf = this.TurnFactor;
                    
                    area  = coupling.Area;
                    sigma = coupling.Conductivity;
                    ls    = coupling.Length;
                    fp    = coupling.ModeledFraction;
                    
                    I = this.BundleCurrentIndex;
                    J = this.CommonModeVoltageIndex;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);
                    pos = 0;
                    for i = 1:Nb
                        pos = pos + 1;
                        rows{pos} = I(i);
                        cols{pos} = I(i);
                        vals{pos} = 0;
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            
                            vals{pos} = vals{pos} - ls * Tf{i}(j)^2 / (sigma(k) * area(k) * fp);
                        end
                        
                        pos = pos + 1;
                        rows{pos} = [I(i), J];
                        cols{pos} = [J, I(i)];
                        vals{pos} = [fp/ls, fp/ls];
                    end
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    kcc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                otherwise
            end
        end
        
        function ccc = Ccc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    % #TODO Add external circuit elements
                    ccc = sparse(coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    % #TODO Add external circuit elements
                    ccc = sparse(coupling.Unknowns, coupling.Unknowns);
                otherwise
            end
        end
        
      	function ff = Ff(this, coupling)
            ff = sparse(coupling.Unknowns, this.Nb);
        end
        
    	function fc = Fc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    Nb  = this.Nb;
                    
                    fp = coupling.ModeledFraction;
                    ls = coupling.Length;

                    rows = this.BundleCurrentIndex;
                    cols = 1:Nb;
                    vals = -fp/ls * ones(1, Nb);

                    fc = sparse(rows, cols, vals, coupling.Unknowns, Nb);
                case CouplingTypes.Static
                    Nb  = this.Nb;
                    
                    fp = coupling.ModeledFraction;
                    ls = coupling.Length;

                    rows = this.BundleCurrentIndex;
                    cols = 1:Nb;
                    vals = fp/ls;

                    fc = sparse(rows, cols, vals, coupling.Unknowns, Nb);
            end
        end
        
     	%% Source current post processing
        function f2i = F2I(this, ~)
            f2i = sparse(this.Nb, this.Nb);
        end
        
        %% Source/Bundle voltage post processing
        function f2v = F2V(this, ~)
            f2v = sparse(this.Nb, this.Nb);
        end
        
        %% Electric field post processing
        function f2e = F2E(this, coupling)
            f2e = sparse(coupling.Elements, this.Nb);
        end
        
        %% Current density post processing
        function f2j = F2J(this, coupling)
            f2j = sparse(coupling.Elements, this.Nb);
        end
        
      	%% Post processing annotations
        function title = getTitle(this, varString)
            switch lower(varString)
                case 'current'
                    title = sprintf('%d-Phase Line', this.Nb);
                case 'voltage'
                    title = sprintf('%d-Phase Phase', this.Nb);
                case 'flux linkage'
                    title = sprintf('%d-Phase Phase', this.Nb);
                otherwise
                    warning('No %s title available', varString);
            end
        end
        
        function labels = getLabels(this, varString)
            switch lower(varString)
                case {'current', 'voltage', 'flux linkage'}
                    Nb = this.Nb;
                    labels = cell(1,Nb);
                    for i = 1:Nb
                        labels{i} = char('A'+i-1);
                    end
                otherwise
                    warning('No %s labels available', varString);
            end
        end
    end
    
    methods (Static)
        function componentOut = newComponent(varargin)
            componentOut = VoltageSource(varargin{:});
        end
    end
end