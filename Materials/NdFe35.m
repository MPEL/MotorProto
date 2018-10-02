classdef NdFe35 < LinearMaterial
    properties (SetAccess = protected)
        Description = 'NdFe35';
        Color = 'm';
        
        RemanentFluxDensity = 1.23;
        Conductivity = 6.67e5;
        Density = 7500;
        Permeability = mu_o * 1.09978;
    end
end