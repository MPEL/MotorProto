classdef HeterogeneousMaterial < MaterialProperty
    %HeterogeneousMaterial.m A class representing a mixture of several homogenous materials
    %
    % HeterogeneousMaterial properties:
    %   BaseProperties - An array of homogeneous MaterialProperty objects
    %   Percentage     - The percent contributed by each of the BaseProperties
    %
    %   HeterogeneousMaterial inherets properties and methods from
    %   MaterialProperty. See the help for MaterialProperty for more
    %   information.
    %
    % See also MaterialProperty
    
    properties (SetAccess = protected)
        Description = 'Mixture'
        Color       = 'g'

        Linear
        Conductivity
        Permeability
        Density
        RemanentFluxDensity
    end
    
    properties
        BaseProperties
        Percentage
    end
    
    methods
        %% Constructor
        function this = HeterogeneousMaterial(varargin)
            this = this@MaterialProperty(varargin{:});
            this = homogenizeProperty(this, 'Linear',              'bool');
            this = homogenizeProperty(this, 'Permeability',        'mean');
            this = homogenizeProperty(this, 'Conductivity',        'mean');
            this = homogenizeProperty(this, 'Density',             'mean');
            this = homogenizeProperty(this, 'RemanentFluxDensity', 'mean');
        end
        
        %% Material Properties
        function s = elementSigma(this, ~)
            baseProps  = this.BaseProperties;
            percentage = this.Percentage;
            N          = numel(baseProps);
            s          = 0;
            for i = 1:N
                s = s + percentage(i) * elementSigma(baseProps(i));
            end
        end
        
        function d = elementDensity(this, ~)
            baseProps  = this.BaseProperties;
            percentage = this.Percentage;
            N          = numel(baseProps);
            d          = 0;
            for i = 1:N
                d = d + percentage(i) * elementDensity(baseProps(i));
            end
        end
        
        %% Fields
        function [B, dBdH] = magnitudeB(this, H)
            baseProps  = this.BaseProperties;
            percentage = this.Percentage;
            N          = numel(baseProps);
            
            B          = zeros(size(H));
            dBdH       = zeros(size(H));
            for i = 1:N
                [b_i, dBdH_i] = magnitudeB(baseProps(i), H);
                
                B    = B    + percentage(i) * b_i;
                dBdH = dBdH + percentage(i) * dBdH_i;
            end
        end
        
        function [M, dMdB] = magnitudeM(this, B)
            baseProps  = this.BaseProperties;
            percentage = this.Percentage;
            N          = numel(baseProps);
            
            M          = zeros(size(B));
            dMdB       = zeros(size(B));
            for i = 1:N
                [m_i, dMdB_i] = magnitudeM(baseProps(i), B);
                
                M    = M    + percentage(i) * m_i;
                dMdB = dMdB + percentage(i) * dMdB_i;
            end
        end
        
        function [H, dHdB] = magnitudeH(this, B)
            baseProps  = this.BaseProperties;
            percentage = this.Percentage;
            N          = numel(baseProps);
            
            H          = zeros(size(B));
            dHdB       = zeros(size(B));
            for i = 1:N
                [h_i, dHdB_i] = magnitudeM(baseProps(i), B);
                
                H    = H    + percentage(i) * h_i;
                dHdB = dHdB + percentage(i) * dHdB_i;
            end
        end
        
        %% Set Homogenized Properties
        function this = homogenizeProperty(this, prop, type)
            baseProps  = this.BaseProperties;
            
            if strcmpi(type, 'mean')
                percentage = this.Percentage;
                N          = numel(baseProps);
                v          = 0;
                for i = 1:N
                    v = v + percentage(i) * baseProps(i).(prop);
                end
            elseif strcmpi(type, 'bool')
                N          = numel(baseProps);
                v          = true;
                for i = 1:N
                    v = v && baseProps(i).(prop);
                end
            end
            
            this.(prop) = v;
        end
    end
end