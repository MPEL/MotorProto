classdef LinearIron < LinearMaterial
    properties (SetAccess = protected)
        Description         = 'LinearIron';
        Color               = 'y';
        
        RemanentFluxDensity = 0;
        Conductivity        = 0;
        Permeability        = mu_o*1000;
        Density             = 7650;
    end
end