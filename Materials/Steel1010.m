classdef Steel1010 < NonlinearMaterial
    properties (SetAccess = protected)
        Description  = 'Steel 1010';
        Color = 'b';

        HData = [0, 238.7, 318.3, 358.1, 437.7, 477.5, 636.6, 795.8, 1114.1, 1273.2, 1591.5, 2228.2, 3183.1, 4774.6, 6366.2, 7957.7, 15915.5, 47746.5, 63662, 79577.5, 159155, 318310, 1909860];
        BData = [0, 0.2003, 0.3204, 0.40045, 0.50055, 0.5606, 0.7908, 0.9310, 1.1014, 1.2016, 1.302, 1.4028, 1.524, 1.626, 1.698, 1.73, 1.87, 2.04, 2.07, 2.095, 2.2, 2.4, 4.4];
        
        BasisDegree = 2;
        
        CoreLossData = [40; 1.3; 1.8];

        RemanentFluxDensity = 0;
        Conductivity = 0;
        Density = 7870;
    end
end