classdef Copper < LinearMaterial
    properties (SetAccess = protected)
        Description         = 'Copper';
        Color               = 'y';
        
        RemanentFluxDensity = 0;
        Conductivity = 5.24e7;
        Permeability = mu_o;
        Density = 8960;
    end
end