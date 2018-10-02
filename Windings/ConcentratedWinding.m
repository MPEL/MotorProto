classdef ConcentratedWinding < Winding
 	properties
        Layers
    end
    
    properties (Dependent)
        WindingDiagram
    end
    
    methods
        function windingDiagram = get.WindingDiagram(this)
            %% Get parameters
            Q = this.Slots;
            p = this.PolePairs;
            m = this.Phases;
            l = this.Layers;
            
            %% Account for maximum number of symmetries
            t = gcd(Q,p);
            Q = Q / t;
            p = p / t;
            
            a_ph = 360 / Q;
            
            if mod(Q,t) == 0
                a_sh = a_ph;
            else
                a_sh = a_ph / 2;
            end
            
            sos = mod((0:(Q-1)) * p, Q) * a_ph;
            
            a = a_ph / 2;
            for i = 1:l
                for j = 1:m
                    
                end
                a = a_ph / 2 + a_sh;
            end

            char(windingDiagram)
            windingDiagram = char(windingDiagram);
        end
    end
end