classdef NonlinearMaterial < MaterialProperty        
    %NonlinearMaterial.m A class representing materials with nonlinear magnetic properties
    %
    % NonlinearMaterial properties:
    %   HData                 - Magnetic field intensity data from the B-H curve 
    %   BData                 - Magnetic flux density data from the B-H curve 
    %   MData                 - Magnetization calculated from HData and BData
    %   CoreLossData          - LossDensity/Frequency/FluxDensity core-loss measurements
    %   CoreLossCoefficients  - Coefficients for the Generalized Steinmetz equation fit to the CoreLossData
    %
    % NonlinearMaterial methods:
    %   calculateCoreLossCoefficients - Calculates the CoreLossCoefficients based on the CoreLossData
    %
    %   NonlinearMaterial inherets properties and methods from
    %   MaterialProperty. See the help for MaterialProperty for more
    %   information.
    %
    % See also MaterialProperty

%{
properties
    %HData - 
    %
    % See also NonlinearMaterial
    HData;
    
    %BData - 
    %
    % See also NonlinearMaterial
    BData;
    
    %MData - 
    %
    % See also NonlinearMaterial
    MData;
    
    %CoreLossData - 
    %
    % See also NonlinearMaterial
    CoreLossData;
    
    %CoreLossCoefficients - 
    %
    % See also NonlinearMaterial
    CoreLossCoefficients;
%}

    properties (Abstract, SetAccess = protected)
        %% Magnetization Curve Data
        HData
        BData
        
        %% Empirical Loss Data
        CoreLossData
    end
    
    properties (SetAccess = protected)
        Linear = false;
        
        %% Nonlinear Data
        MData
        
        %% M-B Curve Basis Function Data
        Bs
        Bt
        Cf
        
        %% Core Losses
        CoreLossCoefficients
    end
    
    methods 
        %% Constructor Methods
        function this = NonlinearMaterial(varargin)
            this = this@MaterialProperty(varargin{:});
            
            %% B-H Curve Preprocessing
            %   Ensure monotonicity of M-B curve which implies, but is not implied by,
            %   monotonicity of the B-H curve. M = B/mu_o - H.
            %
            %   Ensure that dMdB = 0 as B->infinity
            
            B = this.BData;
            if size(B,1) == 1
                B = B';
            end
            
            H = this.HData;
            if size(H,1) == 1
                H = H';
            end
            
            M = B / mu_o - H;
            this.MData = M;
            
            [bs,bt,cf] = mbfit(B,H);
            this.Bs = bs;
            this.Bt = bt;
            this.Cf = cf;
            
            this.CoreLossCoefficients = this.calculateCoreLossCoefficients(this.CoreLossData);
        end
        
        %% Material Properties
        function s = elementSigma(this, ~)
            s = this.Conductivity;
        end
        
        function d = elementDensity(this,~)
            d = this.Density;
        end
        
        %% Nonlinear Function Methods
        function [H, dHdB] = magnitudeH(this, B)
            [H, dHdB] = magnitudeM(this, B);
            H = B / mu_o - H;
            dHdB = 1 / mu_o - dHdB;
        end
        
        function [M, dMdB] = magnitudeM(this, B)
            bs = this.Bs;
            bt = this.Bt;
            cf = this.Cf;
            
            phi    = exp(-B/bt)+exp(-bs/bt);
            dphidx = exp(-B/bt) ./ phi;
            phi    = bt*(log(exp(-bs/bt) + 1) - log(phi));
            
            M    = 0*phi;
            dMdB = M;
            
            for i = 1:numel(cf)
                M    = M + cf(i) * phi.^i;
                dMdB = dMdB + cf(i) * i * phi.^(i-1) .* dphidx;
            end
        end
    end
    
    methods (Static)
        function coefficients = calculateCoreLossCoefficients(coreLossData)
            p = coreLossData(1, :).';
            f = coreLossData(2, :).';
            B = coreLossData(3, :).';

            if (numel(p) == 1 && numel(f) == 1 && numel(B) == 1)
                coefficients = [p, f, B];
            elseif ~(isempty(p) || isempty(B) || isempty(f))
                A = [ones(size(B)), log(f), log(B)];
                b = log(p);

                coefficients    = (A.'*A) \ (A.'*b);
                coefficients(1) = exp(coefficients(1));
            else
                coefficients = [0 0 0];
            end
        end
    end
end