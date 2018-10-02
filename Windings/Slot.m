classdef Slot
    %Slot.m A concrete class representing Slots with a number of conductors
    %   Slot objects are objects with a set of properties to control various
    %   slot configurations.
    %
    % Slot properties:
    %   WindingType         - Configuration of the windings
    %   ConductorType       - How the conductors are modeled
    %   ConductorDynamics   - How the conductor dynamics are simulated
    %   ConductorBoundaries - Window limiting the placement of conductors
    %   Shape               - Outline of the overall slot shape
    %   Turns               - Number of turns (or layers) in the slot
    %
    % Slot methods:
    %   build - Divides the slot into conducting/nonconducting regions
    %
    % See also MotorProto, Model, Assembly, Stator, Wire
    
%{
properties:
 	%WindingType - Configuration of the windings
    %   The WindingType property controls the configure of the winding in the
    %   slot.
    %   
    %   Values:
    %       {Distributed} - Creates a slot with a distributed winding structure
    %
    %       Concentrated  - Creates a slot with a concentrated winding structure
    %
    % Note: At present, there is no implementation for concentrated windings.
    %
    % See also Slot
    WindingType;
    
 	%ConductorType - How the conductors are modeled
    %   The ConductorType property controls how the conductors are modeled. This
    %   property determines whether stranded conductors or a homegenization
    %   technique is used.
    %
    %   Values:
    %       {Homogenized} - Divides the slot into a several solid conducting
    %                       regions and uses a homogenized material based on the
    %                       specified packing factor.
    %
    %       Circular      - Divides the slot into a nonconducting region and
    %                       aseveral conducting regions of solid, circular wires
    %                       connected in parallel.
    %   
    % See the help file for Wire for more information.
    %
    % See also Slot, Wire, HomgenizedConductor, CircularWire
    ConductorType;
    
 	%ConductorDynamics - How the conductor dynamics are simulated
    %   The ConductorDynamics property determines whether or not eddy-current
    %   related phenomena are modeled in a simulation. This property can be set
    %   by a string or using the DynamicsTypes enumeration object.
    %
    % See the help file for DynamicsTypes for more information.
    %
    % See also Slot, DynamicsTypes
    %
    ConductorDynamics;
    
 	%Shape - Outline of the overall slot shape
    %   The Shape property is a Geometry2D object which defines the outline of
    %   the entire slot in x-y plane. The placement of conductors can be
    %   constrained to a certain portion of this region by altering the
    %   ConductorBoundaries property.
    %
    % See also Slot, ConductorBoundaries
    Shape;
    
 	%ConductorBoundaries - Window limiting the placement of conductors
    %   The ConductorBoundaries property is a row vector containing two
    %   positions along the x-axis which define a strip in the x-y plane
    %   oriented parallel to the y-axis. The intersection of this strip and the
    %   Slot object's Shape property define the allowable region for conductors
    %   to be placed.
    %
    %   Example:
    %
    % See also Slot, Shape
    ConductorBoundaries;
    
 	%Turns - Number of turns (or layers) in the slot
    %   The Turns property determines the number of times a group of parallel
    %   conductors passes through the slot. For distributed windings, this
    %	determines the number of winding layers.
    %
    % See also Slot
    Turns;
%}

    properties
        WindingType    = WindingTypes.Distributed
        ConductorType  = ConductorTypes.Homogenized;
        CouplingType   = CouplingTypes.Dynamic;
        AirgapLocation = AirgapLocations.Inside;
        Shape
        Angle
        
        ConductorBoundaries = [-inf inf];
        Layers              = 1;
        Turns               = 1;
    end
    
    properties (SetAccess = private, Dependent)
        ConductorDynamics
    end
    
    properties (Dependent)
        ConductorMaterial
        InsulatorMaterial
    end
    
    properties (Dependent, SetAccess = protected)
        Conductor
    end
    
    properties (Hidden, Access = protected)
        ConductorConfigurations
    end
    
    methods
        %% Constructor
        function this = Slot(varargin)
            if nargin > 0
                for i = 1:2:nargin
                    this.(varargin{i}) = varargin{i+1};
                end
            end
            
            this.ConductorConfigurations = [HomogenizedConductor, CircularWire];
        end
        
        %% Setters
        function this = set.WindingType(this, value)
            if isa(value, 'WindingTypes')
                this.WindingType = value;
            elseif ischar(value)
                this.WindingType = WindingTypes.(value);
            else
                this.WindingType = WindingTypes(value);
            end
        end
        
        function this = set.CouplingType(this, value)
            if isa(value, 'CouplingTypes')
                this.CouplingType = value;
            elseif ischar(value)
                this.CouplingType = CouplingTypes.(value);
            else
                this.CouplingType = WindingTypes(value);
            end
        end
        
        function this = set.ConductorType(this, value)
            if isa(value, 'ConductorTypes')
                this.ConductorType = value;
            elseif ischar(value)
                this.ConductorType = ConductorTypes.(value);
            else
                this.ConductorType = ConductorTypes(value);
            end
        end
            
        function this = set.Shape(this, value)
            assert(isa(value, 'Geometry2D'), 'MotorProto:Slot', 'Slot.Shape must be a Geometry2D object');
            
            this.Shape = value;
        end
        
        function this = set.ConductorMaterial(this, value)
            this.Conductor.ConductorMaterial = value;
        end
        
        function this = set.InsulatorMaterial(this, value)
            this.Conductor.InsulatorMaterial = value;
        end
        
        function this = set.ConductorBoundaries(this, value)
            assert(numel(value) == 2, 'MotorProto:Slot', 'Slot.ConductorBoundaries must be a 2-element vector');
            this.ConductorBoundaries = value;
        end
        
        function this = set.Turns(this, value)
            assert(numel(value) == 1, 'MotorProto:Slot', 'Slot.Turns must be a scalar');
            this.Turns = value;
        end
        
        function this = set.Layers(this, value)
            assert(numel(value) == 1, 'MotorProto:Slot', 'Slot.Layers must be a scalar');
            this.Layers = value;
        end

        %% Getters
        function value = get.Conductor(this)
            conductorType = this.ConductorType;
            value         = this.ConductorConfigurations(conductorType);
        end
        
        function value = get.ConductorDynamics(this)
            switch this.CouplingType
                case CouplingTypes.Static
                    value = DynamicsTypes.Static;
                case CouplingTypes.Dynamic
                    value = DynamicsTypes.Dynamic;
                otherwise
                    error('Unknown Coupling Type %s', char(this.CouplingType));
            end     
        end
        
        %% Other
        function [conductors, nonConductors, locationMatrix] = build(this, label)
            %build - Divides the slot into conducting/nonconducting regions
            %   [C, N, M] = build(S, L) constructs from the Slot object S, an
            %   array C of Region2D objects representing the conductors in the
            %   slot, an array N of Region2D objects representing the 
            %   nonconducting regions of the slot, and a matrix M, representing
            %   the parallel and series connections of the conductors. L is a 
            %   string used as a prefix for the names of the objects in the
            %   arrays C and N.
            %
            %   The columns of M give the parallel connections for each turn,
            %   while the rows of M give the series connections between turns.
            %   For example, M(:, 1) is the first turn consiting of (possibly)
            %   many conductors, while M(1,:) shows the paths that a single 
            %   conductors takes through the slot.
            %
            %   This method is essentially a wrapper for a call to the build
            %   method of a Wire subclass determined by the Slot object's 
            %   Conductor property.
            %
            %   See the help for Wire and its subclasses for more information.
            %
            %   Example: Draw a slot with multiple conductors
            %       r  = [0.5 1];
            %       dr = 0.05;
            %       ns = 72;
            %
            %       S = Slot;
            %       S.ConductorType       = 'Circular';
            %       S.Shape               = slotTemplate(ns,   r(1), r(2), 0.05, ...
            %                                            0.01, 0.5,  0.5,  1,    0);
            %       S.ConductorBoundaries = [r(1) + dr, inf];
            %       S.Turns               = 2;
            %
            %       S.Conductor.ConductorDiameter   = 2 * pi * r(1) / ns * 0.5 / 3;
            %       S.Conductor.InsulationThickness = 2 * pi * r(1) / ns * 0.5 / 18;
            %
            %       L = 'mySlot';
            %
            %       [C, N, M] = build(S, L);
            %
            %       figure
            %       subplot(2,2,1:2);hold on;
            %       title('All Conductors + Slot Outline');
            %       plot([0.55, 0.55],[-0.05, 0.05]);
            %       legend('Conductor Boundary');
            %       plot(C);
            %       wireframe(N);
            %
            %       subplot(2,2,3);
            %       title('Turn 1');
            %       plot(C(M(:,1)));
            %       wireframe(N);
            %
            %       subplot(2,2,4);
            %       title('Conductor 1');
            %       plot(C(M(1,:)));
            %       wireframe(N);
            %
            % See also Slot, Wire, slotTemplate
                    
         	[conductors, nonConductors, locationMatrix] = this.Conductor.build(this.Shape, this.ConductorBoundaries, this.Turns, this.Layers, this.ConductorDynamics, this.CouplingType, this.WindingType, this.Angle, this.AirgapLocation, label);
        end
    end
end