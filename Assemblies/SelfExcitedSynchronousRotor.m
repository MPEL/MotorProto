classdef SelfExcitedSynchronousRotor < PoleAndToothAssembly
    properties
        Poles
        Teeth
     	FieldSlot
        FieldSlots
        TransformerSlot
        TransformerSlots
        OperatingMode = 'synchronous'
    end
    
    properties (Dependent)
        CouplingType
        
        SlotShape
        
        FieldTurns
        FieldWindingType
        FieldLayers
        FieldCouplingType
        
        TransformerTurns
        TransformerWindingType
        TransformerLayers
        TransformerCouplingType

        SpatialSymmetries
        HasHalfWaveSymmetry
        SpaceTimeSymmetries
        GeometricSymmetries
        
        AngularVelocity
    end
    
    methods
        %% Constructor
     	function this = SelfExcitedSynchronousRotor(varargin)
            this = this@PoleAndToothAssembly(varargin{:});
            
            if isempty(this.FieldSlot)
                this.FieldSlot = Slot;
            end
            
            if isempty(this.TransformerSlot)
                this.TransformerSlot = Slot;
            end
            
        	newCircuit = Component.newComponent('FieldWoundTransformer', [this.Name,' Transformer']);
            this = addCircuit(this, newCircuit);
        end
        
        %% Getters
        function value = get.CouplingType(this)
            value = this.Circuits.CouplingType;
        end
        
        function value = get.SpatialSymmetries(this)
            value = this.Poles / 2;
        end
        
        function value = get.HasHalfWaveSymmetry(~)
        	value = true;
        end
        
        function value = get.GeometricSymmetries(this)
            value = this.Teeth;
        end
        
        function value = get.AngularVelocity(this)
            switch this.OperatingMode
                case OperatingModes.Synchronous
                    value = 4 * pi * this.ElectricalFrequency / this.Poles;
                case OperatingModes.Locked
                    value = 0;
            end
        end

        %% Setters
        function set.SlotShape(this, value)
            this.FieldSlot.Shape       = value;
            this.TransformerSlot.Shape = value;
        end
        
        %% Others
        function build(this, fp)
            assert((this.FieldSlots+this.TransformerSlots) == this.Teeth, 'MotorProto', 'The number of field winding slots plus the number of transformer winding slots must equal the total number of rotor teeth');
            
            clean(this);
            
            fSlot = this.FieldSlot;
            tSlot = this.TransformerSlot;

            fCopies = this.FieldSlots * fp;
            tCopies = this.TransformerSlots * fp;
            
            [fConductors, fNonConductors, fLocationMatrix] = build(fSlot, this.Name);
            [tConductors, tNonConductors, tLocationMatrix] = build(tSlot, this.Name);
            
            Nfc = numel(fConductors);
            Ntc = numel(tConductors);
            
            fRegions = [fConductors, fNonConductors];
            tRegions = [tConductors, tNonConductors];
            
            Nfr = numel(fRegions);
            Ntr = numel(tRegions);
            
            fLocationMatrix = repmat(fLocationMatrix, fCopies, 1);
            tLocationMatrix = repmat(tLocationMatrix, tCopies, 1);
            
            fRegionMatrix = fLocationMatrix;
            tRegionMatrix = tLocationMatrix;
            
            da = 2 * pi / this.GeometricSymmetries;
            
            %% Copy Field Winding Slots
            for i = 1:fCopies
                newRegions = copy(fRegions);
                [newRegions.IsUserDefined] = deal(false);
                
                angle = (da * (i - 0.5) + this.InitialAngle) * ones(1, Nfr);
                rotate(newRegions, angle, [0,0]);
                addModelRegion(this, newRegions);
                
                for j = 1:size(fLocationMatrix,2)
                    fLocationMatrix{i,j} = fLocationMatrix{i,j} + Nfc * (i-1);
                    fRegionMatrix{i,j}   = fRegionMatrix{i,j}   + Nfr * (i-1);
                end
                
                for j = 1:size(tRegionMatrix,1)
                    for k = 1:size(tRegionMatrix,2)
                        tRegionMatrix{j,k} = tRegionMatrix{j,k} + Nfr;
                    end
                end 
            end
            
            %% Copy Transformer Winding Slots
            for i = 1:tCopies
                newRegions = copy(tRegions);
                [newRegions.IsUserDefined] = deal(false);
                
                angle = (da * (i + fCopies - 0.5) + this.InitialAngle) * ones(1, Ntr);
                rotate(newRegions, angle, [0,0]);
                addModelRegion(this, newRegions);
                
                for j = 1:size(tLocationMatrix,2)
                    tLocationMatrix{i,j} = tLocationMatrix{i,j} + Ntc * (i-1);
                    tRegionMatrix{i,j}   = tRegionMatrix{i,j}   + Ntr * (i-1);
                end
            end
            
            %% Copy Input Region Geometry
            %   #TODO - Dissallow different slot shapes
            iRegions = this.InputRegions;
            Nir      = numel(iRegions);
            for i = 1:(tCopies+fCopies)
                newRegions = copy(iRegions);
                [newRegions.IsUserDefined] = deal(false);
                
                angle = (da * (i - 0.5) + this.InitialAngle) * ones(1, Nir);
                rotate(newRegions, angle, [0,0]);
                addModelRegion(this, newRegions);
            end
            
            %% Construct Domain Hull
            ang   = 2 * pi * fp;
            rot   = this.InitialAngle;
            rad   = [this.InnerRadius, this.OuterRadius];
          	dHull = Geometry2D.draw('Sector', 'Radius', rad, 'Angle', ang, 'Rotation', rot, 'PlotStyle', {'b'});
            this.DomainHull = copy(dHull);
            dHull = dHull - [this.Regions.Geometry];
            addModelRegion(this, [this.Name,'_DefaultRegion'], dHull, this.DefaultMaterial, DynamicsTypes.Static);
            
            this.ModeledFraction = fp;
            
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    error('#TODO');
                case CouplingTypes.Static
                    Tk = cell(1,2);
                    Tk{1} = [fRegionMatrix{:}];
                    Tk{2} = [tRegionMatrix{:}];

                    Ts = cell(1,2);
                    Ts{1} = ones(size(Tk{1}));
                    Ts{2} = ones(size(Tk{2}));

                    Tf = cell(1,2);
                    Tf{1} = ones(size(Tk{1})) * fSlot.Turns;
                    Tf{2} = ones(size(Tk{2})) * tSlot.Turns;

                    this.Circuits.TurnSets = Tk;
                    this.Circuits.TurnPolarity = Ts;
                    this.Circuits.TurnFactor = Tf;
                otherwise
                    error('Unknown CouplingType %s',char(this.CouplingType));
            end
        end
        
        function previewElement(this)
            domainHull = makeElementDomainHull(this);
            
            if ~isempty(this.InputRegions)
                regionGeometry = this.InputRegions.Geometry;
                domainHull     = domainHull - regionGeometry;
            else
                regionGeometry = [];
            end
            
            % #TODO - Add slots to preview
            
            plot([domainHull, turnGeometry, regionGeometry]);
        end
    end
    
    methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = SelfExcitedSynchronousRotor(varargin{:});
        end
    end
end