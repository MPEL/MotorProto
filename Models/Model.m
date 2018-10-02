classdef Model < matlab.mixin.Copyable
  	%Model.m An abstract interface for all machine models
    %   Model objects are the highest level constructs representing a certain
    %   real world device. Their primary responsibility is coordinating the
    %   interaction and construction of many high level logical units in the
    %   form of Assemby objects.
    %
    % Model methods:
    %   newAssembly    - Adds a new assembly to the model
    %   removeAssembly - Removes an existing assembly from the model
    %   deleteAssembly - Removes an existing assembly from the model and deletes
    %                    the associated object
    %   build          - Constructs the model based on the current configuration
    %   preview        - Preview the fundamental geometric units of the assemblies
    %   plot           - Plots the geometry of the assemblies after running 'build'
    %
    % Model properties:
    %   Assemblies     - An array of Assembly objects
 	%   Mesh           - An array of MeshFactory objects
    %   Components     - An array of Component objects
    %
    % See also MotorProto, RotatingMachineModel, MeshFactory, Assembly, Component
    
%{
properties:
 	%Assemblies - An array of Assembly objects
    %   The Assemblies property is an array of Assembly objects. Model 
    %   objects govern interaction between the assemblies. Each assembly 
    %   object represents a high level machine concept (e.g. 'Stator').
    %
    %   See the help for Assembly and the its subclasses for more information.
    %
    % See also Assembly, newAssembly
    Assemblies;
    
 	%Mesh - An array of MeshFactory objects
    %   Each MeshFactory object is associated with a unique Assembly object
    %   which is found in the Assemblies property. The array of MeshFactory
    %   objects control the discretization of the control the discretization of
    %   the continuous domain defined by the Assemblies.
    %
    %   See the help for MeshFactory and the its related subclasses for more
    %   information.
    %
    % See also MeshFactory
    Mesh;
    
 	%Components - An array of Component objects
    %   Each Assembly object in the Assemblies property array is composed of
    %   several Component objects. All Component objects for each object in the
    %   Assemblies array are listed in the Components property.
    %
    %   See the help for Component and its related subclasses for more
    %   information.
    %
    % See also Component
    Components;
%}
    
    properties (SetAccess = protected)
        Mesh = RotatingMachineMeshFactory.empty(1,0);
    end
    
    properties (SetAccess = private,Dependent)
        Assemblies
        Components
        Mass
    end
    
    methods
        %% Getters
        function value = get.Assemblies(this)
        	value = [this.Mesh.Assembly];
        end
        
        function value = get.Components(this)
        	value = [this.Assemblies.Components];
        end
        
        function value = get.Mass(this)
            value = 0;
            assemblies = this.Assemblies;
            nAssemblies = length(assemblies);
            try
                for i = 1:nAssemblies
                    value = value + assemblies(i).Mass;
                end
            catch ME
                value = [];
            end
        end
        
        %% Assembly Management
     	function assembly = newAssembly(this, assemblyName, assemblyType, varargin)
           	%newAssembly - Adds a new assembly to the model
            % A = newAssembly(myModel, 'Name', 'Type', varargin{:})
            % instantiaties a new assembly named 'Name' of class 'Type' with the
            % property defined in the parameter/value pairs in varargin{:}.
            %
            %   Example: Create add a SynchronousRotor
            %       S = MotorProto('E.G.');
            %       M = sim.Model;
            %       A = mdl.newAssembly('myRotor', 'SynchronousRotor',...
            %                           'Poles', 6, 'InnerRadius', 0.5,...
            %                                       'OuterRadius', 0.75);
            %       M.preview;
            %
            % See also Model, Assembly
            
            if any(isNamed(this,assemblyName))
                error('MotorProto:Model:DuplicateName', 'An assembly named %s already exists', assemblyName);
            end
            
        	assembly = Assembly.newAssembly(assemblyType,  assemblyName, varargin{:});
            
            this.Mesh(end+1) = assembly.newMesh;
        end
        
        function removeAssembly(this, assembly)
           	%removeAssembly - Removes an existing assembly from the model
            % A = removeAssembly(M, 'Name') removes the Assembly object
            % whose name property is 'Name' from the model's Assemblies array
            % but does not delete the object.
            %
            %   Example: Create add a SynchronousRotor
            %       S = MotorProto('E.G.');
            %       M = sim.Model;
            %       A = mdl.newAssembly('myRotor', 'SynchronousRotor',...
            %                           'Poles', 6, 'InnerRadius', 0.5,...
            %                                       'OuterRadius', 0.75);
            %       M.removeAssembly('myRotor');
            %       figure;
            %       subplot(2,1,1); %rotor pole pitch will be visible
            %       A.preview;
            %       subplot(2,1,2);
            %       M.preview;    %will be blank since the model was removed
            %
            % See also Model, Assembly
            
            if ischar(assembly)
                I = isNamed(this, assembly);
            else
                I = (this.Assemblies == assembly);
            end
            
          	this.Mesh(I) = [];
        end
        
        function deleteAssembly(this, assembly)
           	%removeAssembly - Removes an existing assembly from the model
            % A = deleteAssembly(M, 'Name') removes the Assembly object
            % whose name property is 'Name' from the model's Assemblies array
            % and deletes it. Also removes and deletes the associated 
            % MeshFactory object from the Mesh array.
            %
            %   Example: Create add a SynchronousRotor
            %       S = MotorProto('E.G.');
            %       M = sim.Model;
            %       A = mdl.newAssembly('myRotor', 'SynchronousRotor',...
            %                           'Poles', 6, 'InnerRadius', 0.5,...
            %                                       'OuterRadius', 0.75);
            %       M.deleteAssembly('myRotor');
            %       isvalid(A) %false because it was deleted
            %
            % See also Model, Assembly
            
            if ischar(assembly)
                I = isNamed(this, assembly);
            else
                I = (this.Assemblies == assembly);
            end
            
            delete(this.Mesh(I).Assembly);
            delete(this.Mesh(I));
          	this.Mesh(I) = [];
        end
        
        function renameAssembly(this, oldName, newName)
            hasOldName = isNamed(this,oldName);
            
            if any(hasOldName)
                hasNewName = isNamed(this,newName);
                if ~any(hasNewName)
                    this.Assemblies(hasOldName).Name = newName;
                else
                    error('MotorProto:Model', 'An assembly named %s already exists', newName);
                end
            else
                warning('MotorProto:Model', 'No component named %s was found', oldName);
            end
        end
        
        %% Assembly Location
        function I = isNamed(this, name)
            if ~isempty(this.Assemblies)
                names = {this.Assemblies.Name};
                I     = strcmp(name, names);
            else
                I = [];
            end
        end
        
        function n = assemblyIndex(this, name)
            n = find(isNamed(this, name));
        end
        
        %% Plotting
        function plot(this)
            % plot - Plots the geometry of the assemblies after running 'build'
            % plot(M) calls the plot function of each object in the Assemblies
            % array. If the Assemblies property is empty or build has not been
            % called, this method will perform no action.
            %
            %   Example: See RotatingMachineModel/build
            %
            % See also Assembly, preview, RotatingMachineModel/build
            
            if ~isempty(this.Assemblies)
                figure;hold on;
                this.Assemblies.plot;
                grid on;
                xlabel('X [m]');
                ylabel('Y [m]');
            end
        end
        
        function preview(this)
            % preview - Preview the fundamental geometric units of the assemblies
            % preview(M) calls the preview function of each object in the
            % Assemblies array.
            %
            %   Example: See RotatingMachineModel/build
            %
            % See also Assembly, plot, RotatingMachineModel/build

            if ~isempty(this.Assemblies)
                this.Assemblies.preview;
                grid on;
                xlabel('X [m]');
                ylabel('Y [m]');
            end
        end
    end
    
   	methods (Sealed, Access = protected)
      function copyOut = copyElement(this)
         copyOut      = copyElement@matlab.mixin.Copyable(this);
         copyOut.Mesh = copy(this.Mesh);
      end
    end
    
    methods (Abstract)
        this = build(this);
    end
end