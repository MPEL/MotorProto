classdef AsynchronousRotor < PoleAndToothAssembly
    properties
        Slip
    end
    
   	properties (Dependent)        
     	%% New Symmetry Properties
        SpatialSymmetries
        HasHalfWaveSymmetry
        SpaceTimeSymmetries
        GeometricSymmetries
        
        %% Old Symmetry Properties
        SolutionSpaceTimeSymmetry
        SolutionSpaceTimeCoefficients
        AngularVelocity
    end
    
    methods
        %% Constructor
     	function this = AsynchronousRotor(varargin)
            this = this@PoleAndToothAssembly(varargin{:});
        end
        
        %% Getters       	
        function value = get.SpatialSymmetries(this)
            value = this.Poles / 2;
        end
        
        function value = get.HasHalfWaveSymmetry(~)
            value = true;
        end
        
        function value = get.GeometricSymmetries(this)
            value = this.Teeth;
        end
        
        function value = get.SpaceTimeSymmetries(this)
            value = this.Poles / 2;
        end
        
        function value = get.SolutionSpaceTimeSymmetry(this)
            %TODO - Use SpaceTimeSymmetries instead
            value = [this.Poles, inf];
        end
        
        function value = get.SolutionSpaceTimeCoefficients(this)
            value = [-1, 1, 1];
        end
        
        function value = get.Slip(this)
            value = this.Slip;
        end
        
        function value = get.AngularVelocity(this)
        	value = 2 * pi * this.ElectricalFrequency / (this.Poles / 2) * (1 - this.Slip);
        end
    end
    
   	methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = AsynchronousRotor(varargin{:});
        end
    end
end