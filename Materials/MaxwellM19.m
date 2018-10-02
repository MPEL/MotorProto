classdef MaxwellM19 < NonlinearMaterial
    properties (SetAccess = protected)
        Description  = 'MaxwellM19';
        
        Color = 'b';

     	HData = [0,25.46,31.83,47.74,63.66,79.57,159.15,318.3,477.46,636.61,795.77,1591.5,3183,4774.6,6366.1,7957.7,15915,31830];
        BData = [0,0.1,0.15,0.36,0.54,0.65,0.99,1.2,1.28,1.33,1.36,1.44,1.52,1.58,1.63,1.67,1.8,1.9];

        BasisDegree = 2;
        
        CoreLossData = [40; 1.3; 1.8];

        RemanentFluxDensity = 0;
        Conductivity        = 0;
        Density             = 7870;
    end
end