classdef DQOCurrentRegulator < Circuit
    properties (Constant)
        Type = SourceTypes.DQOCurrentRegulator
    end
    
    properties
        ElectricalFrequency = 0;
        InitialAngle        = 0;
     	Phases              = 3;
        
        ConnectionType = ConnectionTypes.Wye;
        
        Idq = [0, 0];
        Kp  = [0, 0];
        Ki  = [0, 0];
    end
    
    properties (Dependent, SetAccess = private)
        BundleVoltageIndex
        BundleCurrentIndex
        CommonModeVoltageIndex
        DQVoltageIndex
        DQCurrentIndex
    end
    
    methods
        function this = DQOCurrentRegulator(varargin)
            this = this@Circuit(varargin{:});
        end
        
        function N = getSize(this) %Matrix factory allocation request
            N = getSize@Circuit(this); %N for strand voltages and currents
            N = N + this.Nb;           %v_i for each bundle current
            N = N + 1;                 %v_o, common mode voltage
            N = N + this.Nb;           %i_i for each bundle voltage
            N = N + 2;                 %v_dq
            N = N + 2;                 %i_dq
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
                    error('#TODO');
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
                    error('#TODO');
            end
        end
        
        function value = get.CommonModeVoltageIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    value = this.Index(end-4);
                otherwise
                    error('#TODO');
            end
        end
        
        function value = get.DQVoltageIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    value = this.Index((end-4+1):(end-2));
                otherwise
                    error('#TODO');
            end
        end
        
        function value = get.DQCurrentIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    value = this.Index((end-2+1):end);
                otherwise
                    error('#TODO');
            end
        end
        
        function build(this, coupling, bcFun)
            matrices = struct('Kfc', [], 'Kcf', [], 'Kcc', [],...
                              'Cfc', [], 'Ccf', [], 'Ccc', [],...
                              'Kdq', [], 'Cdq', [],...
                              'T2k', [], 'K2t', []);
                          
            matrices.Kfc = this.Kfc(coupling);
            matrices.Kcf = this.Kcf(coupling);
            matrices.Kcc = this.Kcc(coupling);
            matrices.Cfc = this.Cfc(coupling);
            matrices.Ccf = this.Ccf(coupling);
            matrices.Ccc = this.Ccc(coupling);
            
            matrices.Kdq = this.Kdq(coupling);
            matrices.Cdq = this.Cdq(coupling);
            
            matrices.T2k = this.T2k(coupling);
            matrices.K2t = this.K2t(coupling);
            
            matrices.Kfc = bcFun(matrices.Kfc, 'both');
            matrices.Kcf = bcFun(matrices.Kcf, 'both');
            matrices.Kcc = bcFun(matrices.Kcc, 'both');
            matrices.Cfc = bcFun(matrices.Cfc, 'both');
            matrices.Ccf = bcFun(matrices.Ccf, 'both');
            matrices.Ccc = bcFun(matrices.Ccc, 'both');
            
            matrices.Kdq = bcFun(matrices.Kdq, 'both');
            matrices.Cdq = bcFun(matrices.Cdq, 'both');
            
            matrices.T2k = bcFun(matrices.T2k, 'rows');
            matrices.K2t = bcFun(matrices.K2t, 'columns');
            
            this.Matrices = matrices;
        end
        
        %% Matrix Functions
        function k = K(this, t, s)
            k = this.Matrices.Kfc + this.Matrices.Kcf * s + this.Matrices.Kcc * s;
            
            f = this.ElectricalFrequency;
            a = -2 * pi * f * t + this.InitialAngle; 
            T = sqrt(2/3) * [ cos(a)    cos(a-2*pi/3)  cos(a+2*pi/3);
                             -sin(a)   -sin(a-2*pi/3) -sin(a+2*pi/3)];
            
            ktk = this.Matrices.T2k * T * this.Matrices.K2t;
            
            k = k + s * ktk + s * ktk';
            
         	k = k + this.Matrices.Kdq * s;
        end
        
        function c = C(this, ~, h, s)
            c = this.Matrices.Cfc / h + this.Matrices.Ccf * s / h + this.Matrices.Ccc * s / h;
            
            c = c + this.Matrices.Cdq * s / h;
        end
        
        %% Matrices
        function kcc = Kcc(this, coupling)
            % #TODO - Add external circuit elements
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    Rk = this.RegionSets;
                    Rs = this.RegionPolarity;
                    
                    [Nb, Nib, Nri, Nss] = this.getBundleSizes;
                    
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
                            vals{pos} = [s, 0 * fp/ls * Nss{i}(j), fp/ls * Nss{i}(j)];
                        end
                    end

                    %% One row for each bundle voltage v_{i}
                    for i = 1:Nb
                        pos = pos + 1;
                        rows{pos} = I(i) * ones(1, Nib(i) + 1);
                        cols{pos} = [J{i}, H(i)];
                        vals{pos} = fp/ls * [Nss{i}, -1];
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
                    
                    pos = pos + 1;
                    rows{pos} = this.DQVoltageIndex;
                    cols{pos} = this.DQCurrentIndex;
                    vals{pos} = fp/ls * [1, 1];
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    kcc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    error('#TODO');
                otherwise
            end
        end
        
        function ccc = Ccc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    % #TODO Add external circuit elements
                    ccc = sparse(coupling.Unknowns, coupling.Unknowns);
                case CouplingTypes.Static
                    error('#TODO');
                otherwise
            end
        end
        
        function t2k = T2k(this, coupling)
            fp = coupling.ModeledFraction;
            ls = coupling.Length;
            
            rows = this.DQVoltageIndex;
            cols = 1:2;
            vals = sqrt(fp/ls) * ones(1,2);
            t2k  = sparse(rows, cols, vals, coupling.Unknowns, 2);
        end
        
        function k2t = K2t(this, coupling)
            fp = coupling.ModeledFraction;
            ls = coupling.Length;
            
            rows = 1:3;
            cols = this.BundleCurrentIndex;
            vals = sqrt(fp/ls) * ones(1,3);
            k2t  = sparse(rows, cols, vals, 3, coupling.Unknowns);
        end
        
        function kdq = Kdq(this, coupling)
            fp = coupling.ModeledFraction;
            ls = coupling.Length;
            N  = coupling.Unknowns;
            
            Iidq = this.DQCurrentIndex;
            
            rows = Iidq;
            cols = Iidq;
            vals = fp / ls * this.Ki;
            kdq  = sparse(rows, cols, vals, N, N);
        end
        
        function cdq = Cdq(this, coupling)
            fp = coupling.ModeledFraction;
            ls = coupling.Length;
            
            rows = [this.DQCurrentIndex, this.DQCurrentIndex];
            cols = [this.DQVoltageIndex, this.DQCurrentIndex];
            vals = fp / ls * [1, 1, this.Kp];
            cdq  = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
        end
        
      	function ff = Ff(~, coupling)
            ff = sparse(coupling.Unknowns, 2);
        end
        
    	function fc = Fc(this, coupling)
            fp = coupling.ModeledFraction;
            ls = coupling.Length;
            
            rows = this.DQCurrentIndex;
            cols = 1:2;
            vals = fp/ls * this.Ki;
            
            fc = sparse(rows, cols, vals, coupling.Unknowns, 2);
        end
        
     	%% Source current post processing
        function f2i = F2I(this, ~)
            f2i = sparse(this.Nb, 2);
        end
        
        %% Source/Bundle voltage post processing
        function f2v = F2V(this, ~)
            f2v = sparse(this.Nb, 2);
        end
        
        %% Electric field post processing
        function f2e = F2E(~, coupling)
            f2e = sparse(coupling.Elements, 2);
        end
        
        %% Current density post processing
        function f2j = F2J(~, coupling)
            f2j = sparse(coupling.Elements, 2);
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
    
  	methods (Sealed)        
        function waveform = f(this, t, s)
            if nargin == 2
                s = 1;
            end
            
%             waveform = s * this.Idq' * ones(1, numel(t));
            waveform = this.Idq' * ones(1, numel(t));
        end
    end
    
    methods (Static)
        function componentOut = newComponent(varargin)
            componentOut = DQOCurrentRegulator(varargin{:});
        end
    end
end