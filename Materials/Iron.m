classdef Iron < NonlinearMaterial
    properties (SetAccess = protected)
        Description  = 'Iron';
        
        Color = 'b';

        HData = [0, 238.73, 795.78, 1591.55, 2387.33, 3978.88, 7957.75, 15915.5, 23873.25, 39788.75, 79577.5, 159155, 318310];
        BData = [0, 0.2500, 0.9250, 1.25000, 1.39000, 1.52500, 1.71000, 1.87000, 1.955000, 2.020000, 2.11000, 2.2250, 2.4300];
        
        BasisDegree = 2;
        
        CoreLossData = [40; 1.3; 1.8];

        RemanentFluxDensity = 0;
        Conductivity = 1e7;
        Density = 7874;
    end
end