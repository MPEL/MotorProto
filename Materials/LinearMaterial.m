classdef LinearMaterial < MaterialProperty
    %LinearMaterial.m A class representing materials with linear magnetic properties
    %
    % LinearMaterial properties:
    %   Permeability - Linear permeability of the material
    %
    %   LinearMaterial inherets properties and methods from
    %   MaterialProperty. See the help for MaterialProperty for more
    %   information.
    %
    % See also MaterialProperty

%{
properties
    %Permeability - 
    %
    % See also LinearMaterial
    Name;
%}
    
    properties (SetAccess = protected)
        Linear = true;
    end
    
  	 properties (Abstract, SetAccess = protected)
        Permeability
     end
     
     methods         
        %% Constructor Methods
        function this = LinearMaterial(varargin)
            this = this@MaterialProperty(varargin{:});
        end
        
        %% Linear Functions
        function [b, dBdH] = magnitudeB(this, h)
            dBdH = this.Permeability;
            b    = dBdH * h ;
        end
        
        function [h, dHdB] = magnitudeH(this, b)
            dHdB = 1 / this.Permeability;
            h    = dHdB * b;
        end
        
        function [m, dMdB] = magnitudeM(this, b)
            dMdB = (1 / mu_o - 1 / this.Permeability);
            m    = dMdB * b;
        end
        
        function s         = elementSigma(this, ~)
            s = this.Conductivity;
        end
        
        function d = elementDensity(this,~)
            d = this.Density;
        end
     end
end