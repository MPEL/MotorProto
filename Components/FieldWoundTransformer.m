classdef FieldWoundTransformer < Circuit
    properties
        Rft = [0,0];
        Lft = [0,0];
        Cft = [0,0];
        
        DiodeVoltages   = [0,0];
        DiodeParameters = [0,0,0,0];
        
        OpenCircuit = true;
    end
    
    properties (Dependent, SetAccess = private)
        BundleVoltageIndex
        BundleCurrentIndex
    end
    
    methods
        function this = FieldWoundTransformer(varargin)
            this        = this@Circuit(varargin{:});
            this.Linear = false;
        end
        
        function N = getSize(this) %Matrix factory allocation request
            N = getSize@Circuit(this); %N for strand voltages and currents
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO')
                case CouplingTypes.Static
                    N = N + this.Nb; %Field/Transformer Winding Currents
                    N = N + this.Nb; %Field/Transformer Winding Voltages
                otherwise
            end
        end
        
        %% Indexing properties
        function value = get.BundleCurrentIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO');
                case CouplingTypes.Static
                    value = this.Index(1:this.Nb);
                otherwise
            end
        end
        
        function value = get.BundleVoltageIndex(this)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO');
                case CouplingTypes.Static
                    Nb    = this.Nb;
                    value = this.Index((Nb+1):(2*Nb));
                otherwise
            end
        end
        
        %% Stiffness matrix functions
        function kcc = Kcc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO');
                case CouplingTypes.Static
                    [Nb, Nk] = this.getBundleSizes;
                    Npp = this.ParallelPaths;
                    
                    Tk = this.TurnSets;
                    Tf = this.TurnFactor;

                    I = this.BundleCurrentIndex;
                    J = this.BundleVoltageIndex;
                    
                    fp    = coupling.ModeledFraction;
                    ls    = coupling.Length;
                    sigma = coupling.Conductivity;
                    area  = coupling.Area;
                    
                    rows = cell(1,0);
                    cols = cell(1,0);
                    vals = cell(1,0);
                    
                    pos = 0;
                    for i = 1:Nb
                        %% Voltage/current coupling
                        for j = 1:Nk(i)
                            k  = Tk{i}(j);
                            
                            pos = pos + 1;
                            rows{pos} = I(i);
                            cols{pos} = I(i);
                            vals{pos} = -Tf{i}(j)^2 / (sigma(k) * area(k) * Npp(i));
                        end
                        
                        %% Bundle external resistances
                        pos = pos + 1;
                        rows{pos} = I(i);
                        cols{pos} = I(i);
                        vals{pos} = -(fp / ls) * this.Rft(i) / Npp(i);
                        
                        %% Bundle voltage
                        pos = pos + 1;
                        rows{pos} = I(i);
                        cols{pos} = J(i);
                        vals{pos} = -fp / ls;
                        
                        %% Capacitor current equation
                        pos = pos + 1;
                        rows{pos} = J(i);
                        cols{pos} = I(i);
                        vals{pos} = -fp / ls;
                    end
                    
                    rows = [rows{:}];
                    cols = [cols{:}];
                    vals = [vals{:}];
                    
                    kcc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                otherwise
            end
        end
         
        %% Mass matrix functions
        function ccc = Ccc(this, coupling)
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO')
                case CouplingTypes.Static
                    fp = coupling.ModeledFraction;
                    ls = coupling.Length;
                    
                    rows = [this.BundleCurrentIndex, this.BundleVoltageIndex];
                    cols = rows;
                    vals = fp * [-this.Lft ./ this.ParallelPaths, this.Cft] / ls;
                    
                    ccc = sparse(rows, cols, vals, coupling.Unknowns, coupling.Unknowns);
                otherwise
            end
        end
        
        %% Forcing vector functions
        function ff = Ff(this, coupling)
            %% NOOP
            %   No exogenous inputs
            ff = sparse(coupling.Unknowns, this.Nb);
        end
        
        function fc = Fc(this, coupling)
            %% NOOP
            %   No exogenous inputs
            fc = sparse(coupling.Unknowns, this.Nb);
        end
        
        %% Bundle current post processing
      	function f2i = F2I(this, ~)
            %% NOOP
            %   No exogenous inputs
            f2i = sparse(this.Nb, this.Nb);
        end
        
        %% Bundle voltage post processing
        function f2v = F2V(this, ~)
            %% NOOP
            %   No exogenous inputs
            f2v = sparse(this.Nb, this.Nb);
        end
        
      	%% Electric Field post processing
      	function f2e = F2E(this, coupling)
            %NOOP, No exogenous inputs
            f2e = sparse(coupling.Elements, this.Nb);
        end
        
      	%% Current Density post processing
      	function f2j = F2J(this, coupling)
            %NOOP, No exogenous inputs
            f2j = sparse(coupling.Elements, this.Nb);
        end
        
      	%% Post processing annotations
        function title = getTitle(~, varString)
            switch lower(varString)
                case 'current'
                    title = 'Rotor Current';
                case 'voltage'
                    title = 'Rotor Voltage';
                case 'flux linkage'
                    title = 'Rotor Flux Linkage';
                otherwise
                    warning('No %s title available', varString);
            end
        end
        
        function labels = getLabels(~, varString)
            switch lower(varString)
                case {'current', 'voltage', 'flux linkage'}
                    labels = {'Field Winding', 'Transformer Winding'};
                otherwise
                    warning('No %s labels available', varString);
            end
        end
    end
    
    methods (Static)
        function componentOut = newComponent(varargin)
            componentOut = FieldWoundTransformer(varargin{:});
        end
    end
    
  	methods (Sealed)        
        function waveform = f(this, t, ~)
            waveform = zeros(this.Nb, numel(t));
        end
        
        function [nonlinearJacobian, nonlinearFunction] = g(this, ~, x, s)
            if ~this.OpenCircuit
                ind = this.Index(end-1:end);%FIXME
                N   = numel(x);
                dp  = this.DiodeParameters;
                op  = numel(dp);
                Vd  = this.DiodeVoltages;

                V  = x(end-1:end);      %FIXME, should depend on local index with periodic boundary conditions
                Tv = [-1 1;-1 -1] / 2;
                V  = Tv * V;

                I    = [0;0];
                dIdV = [0,0;0,0];

                for i = 1:2
                    if V(i) > Vd(2)
                        for j = 1:op
                            I(i) = I(i) * Vd(2) + dp(j);
                        end

                        for j = 1:(op-1)
                            dIdV(i,i) = dIdV(i,i) * Vd(2) + (op-j) * dp(j);
                        end
                        
                        I(i) = I(i) + dIdV(i,i) * (V(i) - Vd(2));
                    elseif V(i) > Vd(1)
                        for j = 1:op
                            I(i) = I(i) * V(i) + dp(j);
                        end
                        
                        for j = 1:(op-1)
                            dIdV(i,i) = dIdV(i,i) * V(i) + (op-j) * dp(j);
                        end
                    end
                end

                Ti = [-1 -1;1 -1];

                I    = Ti * I;
                dIdV = Ti * dIdV * Tv;

                row = [N-1, N];%FIXME
%                 row = ind;
                col = [1,1];
                val = I;
                nonlinearFunction = s * sparse(row, col, val, N, 1);

                %FIXME
%                 row = [ind(1), ind(1), ind(2), ind(2)];
%                 col = [ind(1), ind(2), ind(1), ind(2)];
                row = [N-1, N-1, N, N];
                col = [N-1, N, N-1, N];
                val = [dIdV(1,1), dIdV(1,2), dIdV(2,1), dIdV(2,2)];
                nonlinearJacobian = s * sparse(row, col, val, N, N);
            else
                N = numel(x);
                nonlinearFunction = sparse(N,1);
                nonlinearJacobian = sparse(N,N);
            end
        end
    end
end