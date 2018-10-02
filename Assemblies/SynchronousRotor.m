classdef SynchronousRotor < PoleAssembly
    %SynchronousRotor.m A concrete class representing Rotors intendid to operate at synchronous speed.
    %   SynchronousRotor objects represent assemblies intendid to operate at
    %   synchronous speed. A locked rotor simulation mode is also available by
    %   setting the OperatingMode property.
    %
    % SynchronousRotor properties:
    %   OperatingMode   - Sets one of two operating states of the object
    %   AngularVelocity - Speed of the rotor in radians per second.
    %
    % SynchronousRotor inherits properties and methods PoleAssembly.
    %
    % See the help files for PoleAssembly for more information.
    %
    % See also MotorProto, Model, PoleAssembly
    
%{
properties:
 	%OperatingMode - Sets one of two operating states of the object
    %   The OperatingMode property is used to toggle between one of two
    %   simulation configurations:
    %   
    %   {Synchronous} - The rotor is spinning in synchronicity with the 
    %                   fundamental harmonic. The angular
    %                   velocity is determined by the ElectricalFrequency and
    %                   Poles property.
    %
    %   Locked        - The rotor is locked and the angular velocity is zero.
    %
    % See also SynchronousRotor, AngularVelocity
    OperatingMode;
    
 	%AngularVelocity - Speed of the rotor in radians per second.
    %   The AngularVelocity is the speed of the rotor in radians per second. It
    %   varies depending on the OperatingMode property. The synchronous speed is
    %   defined as ElectricalFrequency * Poles / 2.
    %
    % See also SynchronousRotor, OperatingMode
    ConnectionType;
%}

    properties (Dependent)
        AngularVelocity
    end
    
    properties
        OperatingMode = OperatingModes.Synchronous;
    end
    
    methods
        %% Constructor
     	function this = SynchronousRotor(varargin)
            this = this@PoleAssembly(varargin{:});
        end
        
        %% Getters
        function value = get.AngularVelocity(this)
            switch this.OperatingMode
                case OperatingModes.Synchronous
                    value = 2 * pi * this.ElectricalFrequency / (this.Poles / 2);
                case OperatingModes.Locked
                    value = 0;
            end
        end
    end
    
   	methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = SynchronousRotor(varargin{:});
        end
    end
end