classdef PoleAssembly < RotatingMachineAssembly 
    %PoleAssembly.m An abstract class representing assemblies defined by poles
    %   PoleAssembly objects are assemblies where the fundamental frequency
    %   of the spatial distribution and the number of fundamental geometric 
    %   units coincide.
    %
    % PoleAssembly properties:
    %   Poles - The number of poles of the object
    %
    % PoleAssembly inherits properties and methods RotatingMachineAssembly.
    %
    % See the help files for RotatingMachineAssembly for more information.
    %
    % See also MotorProto, Model, RotatingMachineAssembly, SynchronousRotor
    
%{
properties:
 	%Poles - The number of poles of the object
    %   The Poles property indicates the fundamental spatial frequency of the
    %   excitation and fields in the object. Specifically, half-wave symmetry is
    %   assumed and the fundamental frequency is Poles / 2. One geometric unit 
    %   occupies an angle of 2*pi/Poles.
    %
    % See also PoleAssembly
    Poles;
%}
    properties
        Poles = 2;
    end

  	properties (Dependent)
        %% New Properties
      	SpatialSymmetries
        HasHalfWaveSymmetry
        SpaceTimeSymmetries
        
        GeometricSymmetries
        
        %% Old Properties
        SolutionSpatialFrequency
        SolutionHalfWaveSymmetry
        SolutionTemporalFrequency
        SolutionTemporalSymmetry
        GeometryFrequency
    end
    
    methods
        %% Constructor
     	function this = PoleAssembly(varargin)
            this = this@RotatingMachineAssembly(varargin{:});
        end
        
        %% Getters
        function value = get.SpatialSymmetries(this)
            value = this.Poles / 2;
        end
        
        function value = get.HasHalfWaveSymmetry(~)
            value = true;
        end
        
        function value = get.GeometricSymmetries(this)
            value = this.Poles;
        end
        
        function value = get.SpaceTimeSymmetries(this)
            value = this.Poles / 2;
        end
    end
    
   	methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = PoleAssembly(varargin{:});
        end
    end
end