classdef Assembly < Nameable
    %Assembly.m An abstract class for all Assembly objects
    %   Assembly objects represent high level logical units of a device, e.g.
    %   Stators and Rotors of electrical machines. Their main purpose is
    %   constructing and coordinating the interaction between Component objects,
    %   which represent the sources and regions (geometry/material properties)
    %   associated with the Assembly object. Subclasses of the Assembly class 
    %   specialize behavior and define methods which perform certain actions 
    %   that aide in the construction of a particular type of device.
    %
    % Assembly methods:
    %   addRegion       - Adds a new Region2D object to the Assembly object
    %   removeComponent - Removes a component from the Assembly object
    %   build           - Creates all components of the Assembly
    %   preview         - Display a single geometric unit of the Assembly object
    %   plot            - Displays the geometry of the Assembly object
    %   wireframe       - Display the boundaries of the Assembly object
    %
    % Assembly properties:
    %   InputRegions    - An array of user-defined Region2D objects
    %   Components      - An array of Component objects added after calling build
    %   Regions         - An array of Region2D objects added after calling build
    %   Sources         - An array of Source objects add after calling build
    %   DomainHull      - Outline of the region on which the assembly is defined
    %   DefaultMaterial - The default MaterialProperty object
    %
    % See also MotorProto, Model, MeshFactory, Stator, SynchronousRotor,
    %          Component, Region2D, Source, MaterialProperty
    
%{
properties:
 	%InputRegions - An array of user-defined Region2D objects
    %	The InputRegions property is an array of Region2D objects created by
    %   user calls to the addRegion method. This array defines geometry and
    %   material properties on the Assembly objects fundamental geometric unit.
    %   These regions and the outline of the fundamental geometric unit can be
    %   visualized by calling the preview method.
    %
    % See also Assembly, Component, Region2D, addRegion, preview
    InputRegions;
    
 	%Components - An array of Component objects added after calling build
    %	The Components property is an array of Component objects which define
    %   the modeled portion of the Assembly object. They are created after
    %   calling build and depend on the InputRegions property and possibly other
    %   properties of the assembly object.
    %
    % See also Assembly, Component, InputRegions
    Components;
    
 	%Regions - An array of Region2D objects added after calling build
    %	The Regions property is an array of Region2D objects. The members of the
    %   array are the members of the Components property which are Region2D
    %   objects.
    %
    % See also Assembly, Component, Region2D, Components
    Regions;
    
 	%Sources - An array of Source objects add after calling build
    %	The Sources property is an array of Source objects. The members of the
    %   array are the members of the Components property which are Source
    %   objects.
    %
    % See also Assembly, Component, Source, Components
    Sources;
    
	%DomainHull - Outline of the region on which the assembly is defined
    %	The DomainHull property is a Geometry2D(?) object automatically created
    %   after calling the build method. It represents the region in the 2D-plane
    %   on which the Assembly object is defined.
    %
    % See also Assembly, Geometry2D, build, InputRegions,
    DomainHull;
    
	%DefaultMaterial - The default MaterialProperty object
    %	The DefaultMaterial property is a user specified MaterialProperty
    %   object. When build is called, any region in the DomainHull not covered
    %   by a user defined region will be assigned the DefaultMaterial.
    %
    % See also Assembly, MaterialProperty, build, DomainHull
    DefaultMaterial;
%}
    
    properties (SetAccess = private)
        InputRegions = Region2D.empty(1,0);
        Components   = Component.empty(1,0);        
        IsRegion     = false(1,0);
        IsSource     = false(1,0);
        IsCircuit    = false(1,0);
    end
    
    properties (SetAccess = private, Dependent)
        Regions
        Sources
        Circuits
    end
    
    properties (SetAccess = private, Dependent, Abstract)
        Mass
    end
    
    
    properties (SetAccess = protected)
        DomainHull
    end
    
    properties
        DefaultMaterial = Iron
    end
    
    methods
        %% Constructor
     	function this = Assembly(nameIn, varargin)
            %TODO - Clean up interface
            if nargin ~= 0
                this.Name = nameIn;
                for i = 1:2:(nargin-1)
                    this.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        %% Getters
        function regions = get.Regions(this)
            regions = this.Components(this.IsRegion);
        end
        
        function sources = get.Sources(this)
            sources = this.Components(this.IsSource);
        end
        
        function circuits = get.Circuits(this)
            circuits = this.Components(this.IsCircuit);
        end
        
        %% Setters
        function set.DefaultMaterial(this, materialIn)
            assert(isa(materialIn, 'MaterialProperty'), 'MotorProto:CylindricalComponent:invalidType', 'DefaultMaterial must be a MaterialProperty object');
            this.DefaultMaterial = materialIn;
        end
        
        %% Component Addition
     	function [this, newNames] = addRegion(this, arg1, geometry, material, dynamics, varargin)
            % addRegion - Adds a new Region2D object to the Assembly object
            % addRegion(A, N, G, M, D, S) adds a user defined Region2D object
            % to the assembly object A giving it the string N as a name, 
            % Geometry2D object G, MaterialProperty object M, sets its simulation
            % dynamics to D, and its source to S.
            %
            % addRegion(A, R) adds the user defined region R to the assembly
            % object A.
            %
            % [A, N] = addRegion(A, R) additionally returns a cell array N
            % containing the name the Region2D object R has in the InputRegions
            % property. The region R may be renamed if a duplicate name is
            % detected when attempting to add the region to the InputRegiosn
            % array.
            %
            %   Example: Create a simple Synchronous IPM Machine
            %   S  = MotorProto('E.G.');
            %   M  = S.Model;
            %
            %   %Create Stator Assembly
            %   ST = M.newAssembly('myStator', 'Stator',...
            %                      'Poles', 4, 'Teeth', 24, 'InnerRadius', 0.51,...
            %                      'OuterRadius', 1,...
            %                      'DefaultMaterial', IronExampleMaterial);
            %
            %   %Add a user defined region
            %   G = Geometry2D.draw('Rect2D', 'Position', [0.75, 0],...
            %                       'Length', 0.1, 'Width', 0.1);
            %
            %   ST.addRegion('myRegion', G, Air, 'Static', 'Isolated');
            %
            %   figure;
            %   title('Preview');
            %   M.preview;
            %
            % See also Assembly, Region2D, Geometry2D, MaterialProperty
            
            if nargin == 2
                newRegion = arg1;
            else
                nameIn    = arg1;
                newRegion = Region2D('Name',     nameIn,   'Geometry',      geometry, 'Material', material, ...
                                     'Dynamics', dynamics, 'IsUserDefined', true,     varargin{:});
            end
            [this.InputRegions, newNames] = this.InputRegions.add(newRegion);
        end
        
        function [this, newNames] = addSource(this, arg1, arg2, varargin)
            assert(isempty(this.Sources), 'MotorProto:Assembly', 'Current functionality only supports a single source per assembly');
            if nargin == 2
                newSource = arg1;
            else
                typeIn    = arg1;
                nameIn    = arg2;
                newSource = Component.newComponent(typeIn, nameIn, 'IsUserDefined', true, varargin{:});
            end
            nNewSources                          = numel(newSource);
            [this.Components, newNames]          = this.Components.add(newSource);
            this.IsRegion(end+1:end+nNewSources) = false;
            this.IsSource(end+1:end+nNewSources) = true;
            this.IsCircuit(end+1:end+nNewSources) = true;
        end
        
        function [this, newNames] = addCircuit(this, arg1, arg2, varargin)
            if nargin == 2
                newCircuit = arg1;
            else
                typeIn    = arg1;
                nameIn    = arg2;
                newCircuit = Component.newComponent(typeIn, nameIn, 'IsUserDefined', true, varargin{:});
            end
            nNewCircuits                         = numel(newCircuit);
            [this.Components, newNames]          = this.Components.add(newCircuit);
            this.IsRegion(end+1:end+nNewCircuits) = false;
            this.IsSource(end+1:end+nNewCircuits) = false;
            this.IsCircuit(end+1:end+nNewCircuits) = true;
        end
        
        %% Component Removal
       	function this = removeComponent(this, nameIn)
            % removeComponent - Removes a component from the Assembly object
            % removeComponent(A, N) removes the component with the name in the
            % string N from the Assembly object A. Both the Components property
            % and InputRegions property are searched for a component with the
            % name N.
            %
            % See also Assembly, Components, InputRegions
            
            if ischar(nameIn)
                nameIn = {nameIn};
            end
            
            [this.Components, isRemoved] = remove(this.Components, nameIn);
            this.IsRegion(isRemoved)     = [];
            this.IsSource(isRemoved)     = [];
            this.IsCircuit(isRemoved)    = [];
            this.InputRegions            = remove(this.InputRegions, nameIn);
        end
        
     	function this = clean(this)
            names = {this.Components.Name};
            names = names(~[this.Components.IsUserDefined]);
            [this.Components, isRemoved] = remove(this.Components, names);
            this.IsRegion(isRemoved) = [];
            this.IsSource(isRemoved) = [];
            this.IsCircuit(isRemoved) = [];
            
            names = {this.InputRegions.Name};
            names = names(~[this.InputRegions.IsUserDefined]);
            this.InputRegions = remove(this.InputRegions, names);
        end
        
        %% Index Functions
        function J = convertComponentIndex(this, property, I)
            %TODO - Use convert index instead
            if isempty(I) || (numel(I) == 1 && (I == 0 || I == -1))
                J = [];
            else
                comp  = this.Components(I);
                prop  = this.(property);

                nComp = numel(comp);
                nProp = numel(prop);

                comp  = repmat(comp.', 1, nProp);
                prop  = repmat(prop, nComp, 1);

                [~,J] = find(comp == prop);
                J     = J.';
            end
        end
        
      	function index2 = convertIndex(this, property1, index1, property2)
            if iscolumn(index1)
                wasColumn = true;
                index1    = index1.';
            else
                wasColumn = false;
            end
            
            obj1  = this.(property1)(index1);
            obj2  = this.(property2);
            
            nObj1 = numel(obj1);
            nObj2 = numel(obj2);
            
            obj1  = repmat(obj1.', 1    , nObj2);
            obj2  = repmat(obj2  , nObj1, 1);
            
            [~,index2] = find(obj1 == obj2);
            
            if ~wasColumn
                index2 = index2.';
            end
        end
    end
    
    methods (Access = protected)
     	function [this, newNames] = addModelSource(this, arg1, arg2, varargin)
            if nargin == 2
                newSource = arg1;
            else
                typeIn = arg1;
                nameIn = arg2;
                newSource = ComponentAggregator.newComponent(typeIn, nameIn, 'IsUserDefined', false, varargin{:});
            end
            nNewSources                           = numel(newSource);
            [this.Components, newNames]           = this.Components.add(newSource);
            this.IsRegion(end+1:end+nNewSources)  = false;
            this.IsSource(end+1:end+nNewSources)  = true;
            this.IsCircuit(end+1:end+nNewSources) = true;
        end
        
     	function [this, newNames] = addModelCircuit(this, arg1, arg2, varargin)
            if nargin == 2
                newCircuit = arg1;
            else
                typeIn = arg1;
                nameIn = arg2;
                newCircuit = ComponentAggregator.newComponent(typeIn, nameIn, 'IsUserDefined', false, varargin{:});
            end
            nNewCircuits                           = numel(newCircuit);
            [this.Components, newNames]            = this.Components.add(newCircuit);
            this.IsRegion(end+1:end+nNewCircuits)  = false;
            this.IsSource(end+1:end+nNewCircuits)  = false;
            this.IsCircuit(end+1:end+nNewCircuits) = true;
        end
        
        function [this, newNames] = addModelRegion(this, arg1, geometry, material, dynamics)
            if nargin == 2
                newRegion = arg1;
            else
                nameIn    = arg1;
                newRegion = Region2D('Name',     nameIn,   'Geometry',      geometry, 'Material', material,...
                                     'Dynamics', dynamics, 'IsUserDefined', false);
            end
            nNewRegions                           = numel(newRegion);
            [this.Components, newNames]           = this.Components.add(newRegion);
            this.IsRegion(end+1:end+nNewRegions)  = true;
            this.IsSource(end+1:end+nNewRegions)  = false;
            this.IsCircuit(end+1:end+nNewRegions) = false;
        end
    end
    
    methods (Sealed)
       	%% Visualization Methods
        function plot(this)
            % plot - Displays the geometry of the Assembly object
            % plot(A) calls the plot function of each object in the Regions
            % property of the Assembly object A.
            %
            % Example: See RotatingMachineAssembly/build
            %
            % See the help file of Region2D for more information.
            %
            % See also Assembly, Region2D, preview, wireframe, RotatingMachineAssembly/build
            
            N = numel(this);
            for i = 1:N
                if ~isempty(this(i).Regions)
                    this(i).Regions.plot;
                end
            end
        end
        
        function preview(this)
            % preview - Displays the fundamental geometric unit of the Assembly
            % preview(A) displays the fundamental geometric unit of the Assembly
            % object A. The fundamental geometric unit is the smallest unit of
            % geometry of the A which is tiled to create the entire model.
            %
            % Example: See RotatingMachineAssembly/build
            %
            % See also Assembly, plot, wireframe, RotatingMachineAssembly/build
            
            N = numel(this);
            for i = 1:N
                this(i).previewElement;
            end
        end
        
        function wireframe(this)
            % wireframe - Displays the geometric outline of the Assembly object
            % wireframe(A) calls the wireframe method of each object in the Regions
            % property of the Assembly object A.
            %
            % See the help file of Region2D for more information.
            %
            % Example: See RotatingMachineAssembly/build
            %
            % See also Assembly, Region2D, plot, preview, RotatingMachineAssembly/build
            
            N = numel(this);
            for i = 1:N
                regions  = [this(i).Regions];
                geometry = [regions.Geometry];
                geometry.wireframe;
            end
        end
    end
        
    methods (Abstract)
        build(this, fraction);
        mesh = newMesh(this);
        previewElement(this);
    end
    
	methods (Sealed,Access = protected)
      function copyOut = copyElement(this)
         copyOut            = copyElement@matlab.mixin.Copyable(this);
         copyOut.Components = copy(copyOut.Components);
      end
    end
   
    methods (Static)
        function assemblyOut = newAssembly(assemblyType, varargin)
            assemblyOut = eval(assemblyType);
            if isa(assemblyOut, 'Assembly')
                assemblyOut = assemblyOut.newAssembly(varargin{:});
            else
                error('MotorProto:Assembly:invalidObjectType', '%s is not a recognized Assembly subclass', assemblyType);
            end
        end
    end
end