classdef PoleAndToothAssembly < RotatingMachineAssembly
    %PoleAndToothAssembly.m An abstract class representing assemblies defined by poles and teeth
    %   PoleAndToothAssembly objects are assemblies where the fundamental
    %   frequency of the spatial distribution and the number of fundamental
    %   geometric units do not coincide.
    %
    % PoleAndToothAssembly properties:
    %   Poles - The number of poles of the object
    %   Teeth - The number of teeth of the object
    %
    % PoleAndToothAssembly inherits properties and methods RotatingMachineAssembly.
    %
    % See the help files for RotatingMachineAssembly for more information.
    %
    % See also MotorProto, Model, RotatingMachineAssembly, Stator
    
%{
properties:
 	%Poles - The number of poles of the object
    %   The Poles property indicates the fundamental spatial frequency of the
    %   excitation and fields in the object. Specifically, half-wave symmetry is
    %   assumed and the fundamental frequency is Poles / 2.
    %
    % See also PoleAndToothAssembly
    Poles;
    
 	%Teeth - The number of teeth of the object
    %   The Teeth property indicates the number of fundamental geometric units
    %   required to represent the entire object. One geometric unit occupies an
    %   angle of 2*pi/Teeth.
    %
    % See also PoleAndToothAssembly
    Teeth;
%}
    properties (Abstract)
        Poles
        Teeth
    end
    
    methods
        %% Constructor
     	function this = PoleAndToothAssembly(varargin)
            this = this@RotatingMachineAssembly(varargin{:});
        end
    end
    
   	methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = PoleAndToothAssembly(varargin{:});
        end
    end
end