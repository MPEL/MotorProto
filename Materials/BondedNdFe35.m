classdef BondedNdFe35 < LinearMaterial
    properties (SetAccess = protected)
        Description = 'BondedNdFe35';
        Color = 'm';
        
        RemanentFluxDensity = 0.70;
        Conductivity = 6.67e5;
        Density = 7500;
        Permeability = mu_o * 1.09978;
    end
end