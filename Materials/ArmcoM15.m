classdef ArmcoM15 < NonlinearMaterial
    properties (SetAccess = protected)
        Description  = 'ArmcoM15';
        
        Color = 'b';

        HData = linspace(0,2,21)./(mu_o+(0.8795*exp(2.666*linspace(0,2,21).^2)+94.27).^(-1));
        BData = linspace(0,2,21);
        
        BasisDegree = 2;
        
        CoreLossData = [40; 1.3; 1.8];

        RemanentFluxDensity = 0;
        Conductivity = 0;
        Density = 7870;
    end
end