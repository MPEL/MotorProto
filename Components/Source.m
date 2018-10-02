classdef Source < Circuit
    properties
        ElectricalFrequency = 60;      
        HarmonicNumbers     = 1;
        HarmonicAmplitudes  = 0;
        HarmonicPhases      = 0;
        Phases              = 3;
        
        ConnectionType = ConnectionTypes.Wye;
        
    	Rs = eps;
        Ls = eps;
    end
    
    properties (Abstract, Constant)
        Type
    end
    
    methods
        %% Constructor
        function this = Source(varargin)
            if nargin ~= 0
                if isa(varargin{1}, 'Source')
                    inputObject              = varargin{1};
                    this.Name                = inputObject.Name;
                    this.ElectricalFrequency = inputObject.ElectricalFrequency;
                    this.HarmonicAmplitudes  = inputObject.HarmonicAmplitudes;
                    this.HarmonicPhases      = inputObject.HarmonicPhases;
                    this.Phases              = inputObject.Phases;
                    this.ConnectionType      = inputObject.ConnectionType;
                    this.CouplingType        = inputObject.CouplingType;
                    this.PathSets            = inputObject.PathSets;
                    this.PathPolarity        = inputObject.PathPolarity;
                    this.RegionSets          = inputObject.RegionSets;
                    this.RegionPolarity      = inputObject.RegionPolarity;
                    this.StrandSets          = inputObject.StrandSets;
                    this.TurnSets            = inputObject.TurnSets;
                    this.TurnPolarity        = inputObject.TurnPolarity;
                    for i = 2:2:(nargin-1)
                        this.(varargin{i}) = varargin{i+1};
                    end
                else
                    this.Name = varargin{1};
                    for i = 2:2:(nargin-1)
                        this.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        %% Setters
        function this = set.ConnectionType(this, value)
            if ischar(value)
                this.ConnectionType = ConnectionTypes.(value);
            else
                this.ConnectionType = ConnectionTypes(value);
            end
        end
    end
    
    methods (Sealed)        
        function waveform = f(this, t, h)
            % #FIXME
            m        = this.Phases;
            waveform = zeros(m, numel(t));
            
            if nargin == 2
                n = this.HarmonicNumbers;
                J = 1:size(n,2);
            else
%                 J = ismember(this.HarmonicNumbers, h);
%                 n = this.HarmonicNumbers(:,J);
                n = this.HarmonicNumbers;
                J = 1:size(n,2);
            end
            
            T = 1 / this.ElectricalFrequency;
            w = 2 * pi / T;
            A = this.HarmonicAmplitudes(:,J);
            p = this.HarmonicPhases(:,J);
            s = -(0:(m-1)) / m;
            I = ones(1,numel(t));
            
            if numel(A) == 1
                for i = 1:m
                    waveform(i,:) = A * cos(w * n' * (t - s(i) * T) + p');
                end
            elseif numel(p) > numel(J)
                for i = 1:m
                	waveform(i,:) = A(i,:) * cos((w * n(i,:)') * (t - s(i) * T) + p(i,:)');
                end
            else
                for i = 1:m
                    waveform(i,:) = A * cos(w * n' * (t - s(i) * T) + p');
                end
            end
        end
    end
end