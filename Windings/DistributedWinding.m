classdef DistributedWinding < Winding
 	properties
        CoilSpan
        Layers
    end
    
    properties (Dependent)
        WindingDiagram
    end
    
    methods
        function windingDiagram = get.WindingDiagram(this)
            phases     = this.Phases;
            poles      = this.Poles;
            slots      = this.Slots;
            
            gcdspp     = gcd(poles / 2,slots);
            isBalanced = (mod(slots / phases / gcdspp,2) == 0);
            
            poles      = poles / gcdspp;
            slots      = slots / gcdspp;
            
            layers     = this.Layers;
            coilSpan   = this.CoilSpan;
            SPP        = slots / poles / phases;
            
            %% Error Checking
            assert(SPP >= 1, 'MotorProto:DistributedWinding', 'Distributed windings must have a number of slots per pole per phase greater or equal to one');
            
            assert(isBalanced, 'MotorProto:DistributedWinding', 'For a balanced winding scheme, slots / phases / gcd(poles / 2, slots) must be an integer');
            
            %% Construct boundaries of equivalent integer slot winding distribution
            phaseBoundaries	= [ 0:1:(poles * phases - 1);
                              	1:1:(poles * phases)     ] * 2 * pi / (poles * phases);
            
            %% Determine angular centers of slots
            slotCenters    	= (0:1:(slots - 1)) * 2 * pi / slots + pi / (poles * phases);
            
            %% Construct the winding diagram
           	windingDiagram  = repmat(1:slots,layers,1);
            for i = 1:(poles * phases)
                %% Determine the phase and its negative
                if mod(i,2) == 0
                    phaseLabel = [double('a'), double('A')] + mod(i-1,phases);
                else
                    phaseLabel = [double('A'), double('a')] + mod(i-1,phases);
                end
                
                %% For each layer, shift the conductors by the coil span and flip the polarity
                I = (slotCenters >= phaseBoundaries(1,i)) & (slotCenters < phaseBoundaries(2,i));
                for l = 1:layers
                    windingDiagram(l,I) = phaseLabel(mod(l-1,2)+1);
                    I = [I((end-coilSpan+1):end), I(1:(end-coilSpan))];
                end
            end
            
            windingDiagram = char(windingDiagram);
        end
    end
end