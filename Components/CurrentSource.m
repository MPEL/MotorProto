classdef CurrentSource < Source
    properties (Constant)
        Type = SourceTypes.CurrentSource;
    end
    
    properties (Dependent, SetAccess = private)
        BundleVoltageIndex
        BundleCurrentIndex
    end
    
    methods
        function this = CurrentSource(varargin)
            this = this@Source(varargin{:});
        end
        
        function N = getSize(this) %Matrix factory allocation request
            N = getSize@Source(this); %N for strand voltages and currents
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    N = N + this.Nb; %v_i for each bundle voltage
                case CouplingTypes.Static
                    %Static solver, bundles are independently excited
                otherwise
            end
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
        
        function value = get.BundleCurrentIndex(~)
            value = [];
        end
        
        %% Stiffness matrix functions
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

                    %% One row for each strand current i_{i,m}
                    for i = 1:Nb
                        for j = 1:Nib(i)
                            s = Rs{i}{j};
                            
                            pos = pos + 1;
                            rows{pos} = J{i}(j) * ones(1, Nri{i}(j) + 2);
                            cols{pos} = [K{i}{j}, J{i}(j), I(i)];
                            vals{pos} = [s, 0 * fp/ls * Nss{i}(j), fp/ls * Nss{i}(j) * Npp(i)];
                        end
                    end

                    %% One row for each bundle current i_{i}
                    for i = 1:Nb
                        pos = pos + 1;
                        rows{pos} = I(i)* ones(1, Nib(i));
                        cols{pos} = J{i};
                        vals{pos} = fp/ls * Nss{i} * Npp(i);
                    end

                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    kcc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    %% NOOP for CouplingTypes.Static
                    %   This is the case for static solvers
                    kcc = sparse(coupling.Unknowns, coupling.Unknowns); 
                otherwise
            end
        end
        
        %% Mass matrix functions
        function ccc = Ccc(this,coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    % #TODO Add external circuit elements
                    ccc = sparse(coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    %% NOOP for CouplingTypes.Static
                    %   This is the case for static solvers
                    ccc = sparse(coupling.Unknowns, coupling.Unknowns); 
                otherwise
            end
        end
        
        %% Forcing vector functions
        function ff = Ff(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %NOOP, Input appears in Fc for dynamic solvers
                    ff = sparse(coupling.Unknowns, this.Nb);
                case CouplingTypes.Static
                    Nb = this.Nb;
                    
                    col = 1:Nb;
                    m   = coupling.Unknowns;
                    ff  = this.Circuit2FEA(coupling, col, m, Nb);
                    ff  = -ff;
                otherwise
            end
        end
        
        function fc = Fc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    Nb  = this.Nb;
                    
                    fp = coupling.ModeledFraction;
                    ls = coupling.Length;
                    
                    rows = this.BundleVoltageIndex;
                    cols = 1:Nb;
                    vals = fp/ls * ones(1, Nb);
                    
                    fc = sparse(rows, cols, vals, coupling.Unknowns, Nb); 
                case CouplingTypes.Static
                    %NOOP, Input appears in Ff for static solvers
                    fc = sparse(coupling.Unknowns, this.Nb);
                otherwise
            end
        end
        
        %% Source current post processing
        function f2i = F2I(this, ~)
            Nb = this.Nb;
            f2i = sparse(1:Nb, 1:Nb, ones(1, Nb), Nb, Nb);
        end
        
        %% Source/Bundle voltage post processing
        function f2v = F2V(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %% NOOP
                    %   Voltages are unknowns
                    Nb = this.Nb;
                    rows = 1:Nb;
                    cols = 1:Nb;
                    vals = zeros(1,Nb);
                    
                    f2v = sparse(rows, cols, vals, Nb, Nb);
                case CouplingTypes.Static
                    % #TODO - Add external circuit elements
                    [Nb, Nk] = this.getBundleSizes;
                    
                    Tk = this.TurnSets;
                    Tf = this.TurnFactor;
                    
                    Npp = this.ParallelPaths;
                    
                    area  = coupling.Area;
                    sigma = coupling.Conductivity;
                    ls    = coupling.Length;
                    fp    = coupling.ModeledFraction;
                    
                    rows = 1:Nb;
                    cols = 1:Nb;
                    vals = zeros(1,Nb);
                    for i = 1:Nb
                        for j = 1:Nk(i)
                            k = Tk{i}(j);
                            
                            vals(i) = vals(i) + ls * Tf{i}(j)^2 / (sigma(k) * area(k) * fp * Npp(i)^2);
                        end
                    end
                    
                    f2v = sparse(rows, cols, vals, Nb, Nb);
                otherwise
            end
        end
        
        %% Electric field post processing
        function f2e = F2E(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    f2e = sparse(coupling.Elements, this.Nb);
                case CouplingTypes.Static
                    %Part of E from E_{phi} due to specified current
                    coef = 1./coupling.Conductivity;
                    f2e  = this.Circuit2Field(coupling, coef, 1:this.Nb, coupling.Elements, this.Nb);
                otherwise
            end
        end
        
        %% Current density post processing
        function f2j = F2J(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    f2j = sparse(coupling.Elements, this.Nb);
                case CouplingTypes.Static
                    %J is determined by input currents
                    coef = ones(size(coupling.Conductivity));
                    f2j  = this.Circuit2Field(coupling, coef, 1:this.Nb, coupling.Elements, this.Nb);
                otherwise
            end
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
            componentOut = CurrentSource(varargin{:});
        end 
    end
end
