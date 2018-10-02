classdef RotatingMachineModel < Model
   	%RotatingMachineModel.m The standard class for objects representing Rotating Machines
    %
    % RotatingMachineModel methods:
    %   build - Populates the object's Assemblies property based on the current configuration
    %
	% RotatingMachineModel inheritance:
    %   RotatingMachineModel inherts methods and properties from Model.
    %
    %   See the help for Model for more information.
    %
    % See also Model, RotatingMachineMeshFactory, RotatingMachineAssembly

    properties (Dependent, SetAccess = private)
        SpatialSymmetries
        SpatialFrequency
        TemporalFrequency
        TemporalSubharmonics
        HasHalfWaveSymmetry
        %ModeledFraction
    end
    
    properties (SetAccess = private)
        ModeledFraction = 0;
    end
    
    methods
        %% Getters
        %TODO - Check these methods for correctness in unconventional models
        function value = get.SpatialSymmetries(this)
            assembly = this.Assemblies;
            ss = [assembly.SpatialSymmetries];
            value = ss(1);
            for i = 2:numel(ss)
                value = gcd(value,ss(i));
            end
        end
        
        function value = get.SpatialFrequency(this)
            assemblies  = this.Assemblies;
            value       = assemblies(1).SpatialSymmetries;
            nAssemblies = numel(assemblies);
            for i = 2:nAssemblies
            	value = gcd(value, assemblies(i).SpatialSymmetries);
            end
        end
        
        function value = get.TemporalSubharmonics(this)
            value = this.ModeledFraction;
            value = value * max([this.Assemblies.SpatialSymmetries]);
            if this.HasHalfWaveSymmetry
                value = value * 2;
            end
        end
        
        function value = get.TemporalFrequency(this)
            assembly = this.Assemblies;
            value    = [assembly.ElectricalFrequency];
            dF       = bsxfun(@minus, value.', value);
            isEqual  = abs(dF) < sqrt(eps) * max(abs(value));
            if all(all(isEqual))
                value = value(1);
            end
        end
        
        function value = get.HasHalfWaveSymmetry(this)
            assemblies = this.Assemblies;
            if all([assemblies.HasHalfWaveSymmetry])
                if this.ModeledFraction == 0 || abs(2*this.SpatialSymmetries * this.ModeledFraction - 1) < sqrt(eps)
                    value = true;
                else
                    value = false;
                end
            else
                value = false;
            end 
        end
        
%         function value = get.ModeledFraction(this)
%             value = this.SpatialFrequency;
%             if this.HasHalfWaveSymmetry
%                 value = value * 2;
%             end
%             value = 1 / value;
%         end
        
        %% Build
        function this = build(this, fp)
            % build - Populates the object's Assemblies property based on the current configuration
            % build(M) Constructs the geometry, mesh, and source information of
            % the model based on the current configuration of the objects in the
            % Assemblies array.  The build method attempts to construct the smallest
            % model possible based on a confluence of spatial and temporal
            % symmetries of all the entires in Assemblies property.
            %
            %   Example: Create a simple Synchronous IPM Machine
            %   S  = MotorProto('E.G.');
            %   M  = S.Model;
            %   %Create Rotor
            %   RT = M.newAssembly('myRotor', 'SynchronousRotor',...
            %                      'Poles', 4, 'InnerRadius', 0.25,...
            %                                  'OuterRadius', 0.5,...
            %                      'DefaultMaterial', IronExampleMaterial);
            %
            %  %Add Permanent Magnet
            %   PM = Geometry2D.draw('Rect', 'Width', 0.5, 'Length', 0.03,...
            %                        'Base', 'Center', 'Position', [0.35, 0]);
            %
            %   RT.addRegion('Magnet', PM,  PermanentMagnetExampleMaterial,...
            %                'Dynamic', 'Isolated');
            %
            %   %Create Stator
            %   ST = M.newAssembly('myStator', 'Stator',...
            %                      'Poles', 4, 'Teeth', 24, 'InnerRadius', 0.51,...
            %                      'OuterRadius', 1,...
            %                      'DefaultMaterial', IronExampleMaterial);
            %
            %   %Add a slot using template
            %   [slotBody, slotFront] = slotTemplate(24,  0.51, 1, 0.05, 0.01, ...
            %                                        0.4, 0.5,  1, 'Auto');
            %
            %   ST.Slot.Shape = slotBody;
            %   ST.Slot.Turns = 2;
            %
            %   ST.addRegion('SlotFront', slotFront, Air, 'Static', 'Isolated');
            %
            %   figure;
            %   title('Preview');
            %   M.preview;
            %
            %   M.build;
            %   figure;
            %   title('One Pole Plot After Calling build(M)');
            %   M.plot;
            %
            %   figure;
            %   title('Mesh with default settings');
            %   M.Mesh.plot;
            %
            % See also MotorProto, RotatingMachineModel
            
            if nargin == 1
                fp = 1 / this.SpatialSymmetries;
                if this.HasHalfWaveSymmetry
                    fp = fp / 2;
                end
            end
            
            assemblies  = this.Assemblies;
          	nAssemblies = numel(assemblies);
            for i = 1:nAssemblies
                assemblies(i).build(fp);
            end
            
            this.ModeledFraction = fp;
        end
    end    

    methods (Sealed)
%         function [t, h, dft_fun, idft_fun] = discretizeTimeAxis(this, N)
%             assemblies  = this.Assemblies;
%             nAssemblies = numel(assemblies);
%             fe = zeros(nAssemblies,1);
%             fr = zeros(nAssemblies,1);
%             for i = 1:nAssemblies
%                 fe(i) = assemblies(i).ElectricalFrequency;
%                 fr(i) = assemblies(i).AngularVelocity / pi / assemblies(i).Poles;
%             end
%             
%             df  = zeros(2*nAssemblies,1);
%             for i = 1:ne
%                 for j = 1:nr
%                     df(nr*(i-1)+j) = abs(fe(i)-fr(j));
%                 end
%             end
%             
%             f = [];
%             isUnique
%             for i = 1:(ne+nr-1)
%                 for j = i:(ne+nr)
%                 end
%             end
%         end
        
        function [t, h] = getTimePoints(this, Nt)
            f  = this.TemporalFrequency / this.TemporalSubharmonics;
            T  = 1 / f;
            Nt = Nt * this.TemporalSubharmonics;
            
            assemblies  = this.Assemblies;
            nAssemblies = numel(assemblies);
            
            Nh      = ceil(Nt / 2);
            NhSynch = floor((Nh + 2) / 6) * 6;
            NhStat  = NhSynch + 1;
            hStat   = 1:2:NhStat;
            hSynch  = 0:6:NhSynch;
            Nt      = (2 * NhSynch + 6) + 1;
            t       = linspace(0, 1, Nt) * T;
            h       = cell(1, nAssemblies);
            
            for i = 1:nAssemblies
                if assemblies(i).AngularVelocity == 0
                    h{i} = hStat;
                else
                    h{i} = hSynch;
                end
            end
        end
    end
end