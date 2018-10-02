classdef (Sealed) MotorProto < handle
    %MotorProto.m Root for the Electric Machine Rapid Prototyping Toolbox
    %   M = MotorProto is used to initialize the Electric Machine Simulation
    %   and Prototyping ToolBox.  From this object, one can define parameters,
    %   create stator and rotor assemblies, and configure simulations.
    %
    %   M = MotorProto('MP Example') puts a reference to the global
    %   MotorProto object in M and names it "MotorProto Example".
    %
    %   Example: Only one MotorProto object exists at any time
    %       M1 = MotorProto('First Instance')
    %       M2 = MotorProto('Second Instance')
    %       M1 == M2
    %
    % MotorProto methods:
    %   configureAlgorithm - Setup/change the selected solver
    %   run                - Runs a simulation using the active model and algorithm
    %
    % MotorProto properties:
    %   Name       - Identifier for the simulation
    %   Model      - The active machine model
    %   Mesh       - An array of meshes associated with the model
    %   Assemblies - An array of assemblies associated with the model
    %   Components - An array of components associated with the assemblies
    %   Algorithm  - The active solver
    %   Solution   - Results from the most recent call of the 'run' method
    %
    % See also MotorProtoLicense, handle, Model, MeshFactory, Assembly, Components
    
    %	Copyright 2011 Jason Pries
    %	$Revision 0.0.0.2 $
    
%{
properties
    %Name - Identifier for the simulation setup
    %   Name is a descriptive property which may be used to catalog the purpose 
    %   of the simulation.
    %
    % See also MotorProto
    Name;
    
    %Model - The active machine model
    %   The Model property is a single object which is a sub-class of the
    %   Model class.
    %   
    %   See the help for Model for more information.
    %
    % See also MotorProto, Model
    Model;
    
    %Mesh - An array of meshes associated with the model
    %   The mesh property is an array of objects which are sub-classes of the
    %   MeshFactory class.
    %   
    %   See the help for MeshFactory for more information.
    %
    % See also MotorProto, MeshFactory
    Mesh;
    
   	%Assemblies - An array of assemblies associated with the model
    %   The Assemblies property is an array of objects which are sub-classes 
    %   of the Assembly class.
    %   
    %   See the help for Assembly for more information.
    %
    % See also MotorProto, Assembly
    Assemblies;
    
	%Components - An array of components associated with the assemblies
    %   The Components property is an array of objects which are sub-classes 
    %   of the Component class.
    %   
    %   See the help for Component for more information.
    %
    % See also MotorProto, Component
    Components;
    
	%Algorithm - The active solver
    %   The Algorithm property is an object which is a sub-class of the Solver
    %   class.
    %   
    %   See the help for Solver for more information.
    %
    % See also MotorProto, Solver
    Algorithm;
    
	%Solution - Results from the most recent call of the MotorProto run method
    %   The Solution property is an object which is a sub-class of the Solution
    %   class. This propert is set automatically using the results from the most
    %   recent call to run.
    %
    % See also MotorProto, Solution, run, Solver
    Solution;
%}
    
    %% Properties
    properties
        Name
    end
    
	properties (SetAccess = private)
        Created
        Model        
        Algorithm
        Solution
    end
    
    properties (SetAccess = private, Dependent)
        Mesh
        Assemblies
        Components
    end
    
    %% Methods
    methods
        %% Singleton Class Constructor
        function THIS = MotorProto(nameIn)
            persistent UNIQUE_INSTANCE
            if isempty(UNIQUE_INSTANCE)
                warning off 'MotorProto:Verbose'
                
                if nargin == 0
                    nameIn = '';
                end
                THIS.Name       = nameIn;
                THIS.Created    = datestr(now);
                THIS.Model      = RotatingMachineModel;
                
                UNIQUE_INSTANCE = THIS;
            else
                THIS = UNIQUE_INSTANCE;
                if nargin ~=0
                    THIS.Name = nameIn;
                end
            end
        end
        
        %% Setters/Getters
        function value = get.Mesh(this)
            value = this.Model.Mesh;
        end
        
        function value = get.Assemblies(this)
            value = this.Model.Assemblies;
        end
        
        function value = get.Components(this)
            value = this.Model.Components;
        end
        
        %% Other Methods
        function algorithm = configureAlgorithm(this, solverType, varargin)
            %configureAlgorithm - Setup/change the selected solver
            % A = configureAlgorithm(MP, S, 'parameter1', value1, ...) 
            % changes the active solver stored in the MP.Algorithm property
            % to the type specified in S and sets its properties to those
            % specified in the parameter/value pairs.
            %
            % See the help for Solver and the related subclasses for details on
            % the available solvers and their properties.
            %
            % See also MotorProto, Solver
            
            algorithm      = Solver.configureSolver(solverType, varargin{:});
            this.Algorithm = algorithm;
        end
        
        function solution = run(this, x0)
            %run - Runs the simulation using the active model and algorithm
            % S = run(MP) runs the simulation using the model and solver
            % specified in the MotorProto object MP and stores the solution in
            % S. The most recent result of calling run(MP) is also stored in
            % MP.Solution.
            %
            % S = run(MP, X) performs the simulation using X as an initial
            % condition.
            %
            % See the IPM_Machine_Tutorial script for a detailed description of
            % setting up and running a simulation.
            %
            % See also MotorProto, Solution, Solver, IPM_Machine_Tutorial
            
            if nargin == 2
                this.Solution = this.Algorithm.solve(this.Model, x0);
            else
                this.Solution = this.Algorithm.solve(this.Model);
            end
            
            solution = this.Solution;
        end
    end
    
    methods (Static)
    	function name = whatIs(thisVariable)
            switch thisVariable
                case 'A'
                    name = 'Magnetic Vector Potential';
                case 'AverageLosses'
                    name = 'Average Losses';
                case 'B'
                    name = 'Magnetic Flux Density';
                case 'E'
                    name = 'Electric Field Intensity';
                case {'lambda', 'FluxLinkage'}
                    name = 'Flux Linkage';
                case 'H'
                    name = 'Magnetic Field Intensity';
                case {'i', 'Current'}
                    name = 'Current';
                case 'J'
                    name = 'Electric Current Density';
                case 'l'
                    name = 'Losses';
                case 'AverageLossDensity'
                    name = 'Average Loss Density';
                case 'LossDensity'
                    name = 'Loss Density';
                case 'AverageConductionLossDensity'
                    name = 'Average Conduction Loss Density';
                case 'ConductionLossDensity'
                    name = 'Conduction Loss Density';
                case 'CoreLossDensity'
                    name = 'Core Loss Density';
                case 'AverageCoreLossDensity'
                    name = 'Average Core Loss Density';
                case 'M'
                    name = 'Magnetization';
                case {'tau', 'Torque'}
                    name = 'Torque';
                case {'v', 'Voltage'}
                    name = 'Voltage';
                otherwise
                    warning('MotorProto:whatIs', '%s is an unknown variable', thisVariable);
                    name = thisVariable;
            end
        end
        
        function units = unitsOf(thisVariable)
            switch thisVariable
                case {'A',MotorProto.whatIs('A')}
                    units = 'T-m';
                case {'AverageLosses',MotorProto.whatIs('AverageLosses')}
                    units = 'W';
                case {'B',MotorProto.whatIs('B')}
                    units = 'T';
                case {'E',MotorProto.whatIs('E')}
                    units = 'V/m';
                case {'H',MotorProto.whatIs('H')}
                    units = 'A/m';
                case {'i',MotorProto.whatIs('i')}
                    units = 'A';
                case {'J',MotorProto.whatIs('J')}
                    units = 'A/m^2';
                case {'l',MotorProto.whatIs('l')}
                    units = 'W';
                case {'lambda','FluxLinkage',MotorProto.whatIs('FluxLinkage')}
                    units = 'V-s';
                case {'M',MotorProto.whatIs('M')}
                    units = 'A/m';
                case {'AverageLossDensity',MotorProto.whatIs('AverageLossDensity')}
                    units = 'W/m^3';
                case {'AverageCoreLossDensity',MotorProto.whatIs('AverageCoreLossDensity')}
                    units = 'W/m^3';
                case {'AverageConductionLossDensity',MotorProto.whatIs('AverageConductionLossDensity')}
                    units = 'W/m^3';
                case {'tau',MotorProto.whatIs('tau')}
                    units = 'N-m';
                case {'v',MotorProto.whatIs('v')}
                    units = 'V';
                otherwise
                    warning('MotorProto:unitsOf', '%s is an unknown variable',thisVariable);
                    units = thisVariable;
            end
        end
    end
    
    methods (Static, Hidden)
        function [s,fs] = M
            %M - Play the University of Michigan fight song
            %   [S,Fs] = Motorproto.M returns a matrix s containing stereo audio
            %   data for the University of Michigan fight song, Hail to the 
            %   Victors and the corresponding sample rate fs. The song will be
            %   played if an audio device is found. If not, some of the lyrics
            %   will be printed to the console.
            %
            % See also audiovideo
            
        	deviceInfos = audiovideo.internal.audio.DeviceInfo.getDevices;
            outputInfos = deviceInfos([deviceInfos.NumberOfOutputs] > 0);
            noAudio     = isempty(outputInfos); 
            if noAudio
                sprintf(['Hail! to the victors valiant\n',...
                            'Hail! to the conquering heroes\n',...
                            'Hail! Hail! to Michigan\n',...
                            'The leaders and best!'])
            else
                try
                    [s,fs] = wavread('httv');
                    player = audioplayer(s, fs, 16);
                    playblocking(player);
                catch
                    sprintf(['Hail! to the victors valiant\n',...
                                'Hail! to the conquering heroes\n',...
                                'Hail! Hail! to Michigan\n',...
                                'The leaders and best!'])
                end
            end
        end
    end
end