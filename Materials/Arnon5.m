classdef Arnon5 < NonlinearMaterial
    properties (SetAccess = protected)
        %% Basic material information
        Description  = 'Arnon5';
        Color = 'b';
        
        %% HB Curve Data
        HData = [0.00 0.70 1.00 1.500 2.00 3.00 4.00 6.00 15.0 25.0] * 1000 / 4 / pi;
        BData = [0.00 0.28 0.53 0.875 1.05 1.22 1.29 1.35 1.45 1.50];
        
        BasisDegree = 2;
        
        %% Empirical Loss Data
        CoreLossData = [[2.00 4.00 7.00 9.00 6.00 8.00 10.0 20.0 8.00 50.0 5.00 8.00 60.0 90.0 80.0 20.0 60.0 70.0 60.0 90.0 60.0 100 ] * 1.7e4;
                        [400  400  400  400  1000 1000 1000 1000 1500 1500 2000 2000 2000 2000 2500 3000 3000 3000 3500 3500 4000 4000]
                        [0.60 0.90 1.20 1.40 0.60 0.70 0.80 1.20 0.50 1.40 0.30 0.40 1.20 1.50 1.20 0.50 0.90 1.00 0.80 1.00 0.70 0.90]];

        %% Remanent Flux Density
        RemanentFluxDensity = 0;
        Conductivity        = 0;
        Density             = 7650;
    end
end