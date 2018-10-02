classdef Stator < PoleAndToothAssembly
    %Stator.m A concrete class representing Stators with distributed windings
    %   Stator objects are assemblies representing integer stators with distrubuted
    %   windings having an integer number of slots per pole per phase.
    %
    % Stator properties:
    %   Slot           - An object representing the conductor layout and shape
    %                    of a single slot of the stator.
    %   ConnectionType - Indicates how the windings of the Stator are connected
    %   SourceType     - Indicates how the windings of the Stator are excited
    %
    % Stator inherits properties and methods PoleAndToothAssembly.
    %
    % Stator provides and interface for properties of Slot.
    %
    % See the help files for PoleAndToothAssembly and Slot for more information.
    %
    % See also MotorProto, Model, PoleAndToothAssembly, Slot, ConnectionTypes
    
%{
properties:
 	%Slot - An object representing the conductor layout and shape of a single slot of the stator.
    %   A Stator object's Slot property is a Slot object. The slot object
    %   specifies the shape of the slot and controls how the conductors are
    %   layed out within that area.
    %
    %   See the help for Slot for more information.
    %
    % See also Stator, Slot
    Slot;
    
 	%ConnectionType - Indicates how the windings of the Stator are connected
    %   The ConnectionType property indicates the configuration of the Stator
    %   windings, e.g. Wye/Delta. The property can be set using a string or
    %   through the ConnectionTypes enumeration object.
    %
    %   See the help for ConnectionTypes for more information.
    %
    % See also Stator, ConnectionTypes
    ConnectionType;
    
 	%SourceType - Indicates how the windings of the Stator are excited
    %   The SourceType property indicates the type of source connected to the
    %   stator windings, e.g. Current/Voltage. The property can be set using a
    %   string or through the SourceTypes enumeration object.
    %
    %   See the help for SourceTypes for more information.
    %
    % See also Stator, SourceTypes
    SourceTypes;
%}  
    properties
        Poles = 4;
        Teeth = 12;
        Slot
    end
    
    properties (Dependent)
        Phases
        SourceType        
        ConductorDynamics
        ConnectionType
        CouplingType
        ConductorMaterial
        InsulatorMaterial

        WindingType
        Layers
        Turns
        ParallelPaths
        
        SpatialSymmetries
        HasHalfWaveSymmetry
        SpaceTimeSymmetries
        GeometricSymmetries
        
        AngularVelocity
    end
    
    methods
        %% Constructor
     	function this = Stator(varargin)
            this = this@PoleAndToothAssembly(varargin{:});
            
            if isempty(this.Circuits)
                this.SourceType = SourceTypes.CurrentSource;
            end
            
            if isempty(this.Slot)
                this.Slot = Slot;
            end
            
           	this.Slot.Angle = 2*pi/this.Teeth;
        end
        
        %% Getters
        function value = get.Phases(this)
            value = this.Circuits.Phases;
        end
        
        function value = get.SourceType(this)
            value = this.Circuits.Type;
        end
        
        function value = get.ConnectionType(this)
            value = this.Circuits.ConnectionType;
        end
        
        function value = get.CouplingType(this)
            cts = this.Slot.CouplingType;
            ctc = this.Circuits.CouplingType;
            if cts ~= ctc
                error('The stator slot and stator source have specified different coupling types');
            else
                value = cts;
            end
        end
        
        function value = get.ConductorMaterial(this)
            value = this.Slot.ConductorMaterial;
        end
        
    	function value = get.InsulatorMaterial(this)
            value = this.Slot.InsulatorMaterial;
        end
       
        function value = get.WindingType(this)
            value = this.Slot.WindingType;
        end
       
        function value = get.Layers(this)
            value = this.Slot.Layers;
        end
       
        function value = get.Turns(this)
            value = this.Slot.Turns;
        end
        
        function value = get.ParallelPaths(this)
            value = this.Circuits.ParallelPaths;
        end

        function value = get.SpatialSymmetries(this)
            switch this.WindingType
                case WindingTypes.Distributed
                    value = this.Poles / 2;
                case WindingTypes.Concentrated
                    value = gcd(this.Poles, this.Teeth);
                    if mod(value, 2) == 0
                        value = value / 2;
                    end
                otherwise
                    error('No Implementation');
            end
        end
        
        function value = get.HasHalfWaveSymmetry(this)
            switch this.WindingType
                case WindingTypes.Distributed
                    value = true;
                case WindingTypes.Concentrated
                    g         = gcd(this.Poles, this.Teeth);
                    oddPoles  = (mod(this.Poles / g, 2) == 1);
                    if oddPoles
                        evenTeeth = (mod(this.Teeth / g, 2) == 0);
                        if evenTeeth || (this.Layers == 2)
                            value = true;
                        else
                            value = false;
                        end
                    else
                        value = false;
                    end
                otherwise
                    error('No Implementation');
            end
        end
        
        function value = get.GeometricSymmetries(this)
            value = this.Teeth;
        end
        
        function value = get.AngularVelocity(~)
            value = 0;
        end
        
        %% Setters
        function this = set.Phases(this, value)
            this.Circuits.Phases = value;
        end

     	function this = set.SourceType(this, value)
            if isa(value, 'SourceTypes')
                newType = value;
            elseif ischar(value)
                newType = SourceTypes.(value);
            else
                newType = SourceTypes(value);
            end
            
            oldSource = this.Circuits;
            if isempty(oldSource)
                newSource = Component.newComponent(char(newType), [this.Name,' Source']);
                this = addSource(this, newSource);
            elseif ~(oldSource.Type == newType)
                newSource = Component.newComponent(char(newType), [this.Name,' Source']);
                this = removeComponent(this, oldSource.Name);
                this = addCircuit(this, newSource);
            end
            
            if ~isempty(this.Slot)
                newSource.CouplingType = this.Slot.CouplingType;
            end
        end
        
        function this = set.ConnectionType(this, value)
            this.Circuits.ConnectionType = value;
        end
        
        function this = set.CouplingType(this, value)
            this.Slot.CouplingType = value;
            this.Circuits.CouplingType = value;
        end
        
     	function this = set.ConductorMaterial(this, value)
            this.Slot.ConductorMaterial = value;
        end
        
        function this = set.InsulatorMaterial(this, value)
            this.Slot.InsulatorMaterial = value;
        end
               
        function this = set.WindingType(this, value)
            this.Slot.WindingType = value;
        end
               
        function this = set.Layers(this, value)
            this.Slot.Layers = value;
        end
               
        function this = set.Turns(this, value)
            this.Slot.Turns = value;
        end
        
        function this = set.ParallelPaths(this, value)
            this.Circuits.ParallelPaths = value;
        end
        
        function this = set.Slot(this, value)
            assert(isa(value, 'Slot'), 'Stator.Slot must be a Slot object');
            this.Slot = value;
        end
        
        function this = set.Teeth(this, value)
            this.Teeth = value;
            this.Slot.Angle = 2*pi/value;
        end
        
        %% Others
        %   #TODO, factor common code out of buildDistributedWindingStator/buildConcentratedWindingStator into build
        %   - or - make separate classes for DistributedWindingStator and ConcentratedWindingStator
        %
        %   #TODO, everything works, but the winding code could be cleaned up
        function build(this, fp)
            this = clean(this);
            switch this.WindingType
                case WindingTypes.Distributed
                    buildDistributedWindingStator(this, fp);
                case WindingTypes.Concentrated
                    buildConcentratedWindingStator(this, fp);
                otherwise
                    error('Unknown winding type %s',char(this.WindingType));
            end
        end

        function buildDistributedWindingStator(this, fp)
            %% Check Configuration
          	nSlots       = this.Teeth;
            Np           = this.Poles;
            turnsPerSlot = this.Slot.Turns;
            assert(mod(nSlots / Np, 3) == 0, 'MotorProto:StatorComponent', 'The number of teeth per pole must be an integer multiple of 3. The current value is %d', nSlots/Np);

            if nargin < 2
                if this.HasHalfWaveSymmetry
                    fp = 1 / this.SpatialSymmetries / 2;
                else
                    fp = 1 / this.SpatialSymmetries;
                end
            end
            
          	validateattributes(fp, {'numeric'}, {'scalar'});
            nCopies = this.GeometricSymmetries * fp;
            
          	assert(abs(nCopies-round(nCopies)) < sqrt(eps), 'MotorProto:RotatingMachineAssembly', 'ModelFraction must be an integer multiple of 1 / GeometricSymmetries');
              
            nCopies = round(nCopies);
            
            assert(nCopies > 0, 'MotorProto:RotatingMachineAssembly', 'GeometricSymmetries * ModeledFraction >= 1 is required');
            
          	%% Build Slot
            [conductors, nonConductors, locationMatrix] = build(this.Slot, this.Name);

         	regions     = [conductors, nonConductors, this.InputRegions];
            nRegions    = numel(regions);
            nConductors = numel(conductors);
            
            Nb = 3;                                         %Three bundles/phases
            Nc = nConductors * nSlots;                     	%Total number of conductor regions
            sp = 2 * mod(fp*Np-1, 2) - 1;                   %Periodicity coefficient

            %% Copy the regions and rotate them
            locationMatrix = repmat(locationMatrix, nCopies, 1);
            regionMatrix   = locationMatrix;
            
            da = 2 * pi / this.GeometricSymmetries;
            for i = 1:nCopies
                newRegions = copy(regions);
                [newRegions.IsUserDefined] = deal(false);
                
                angle = (da * (i - 0.5) + this.InitialAngle) * ones(1, nRegions);
                rotate(newRegions, angle, [0,0]);
                addModelRegion(this, newRegions);
                
                for j = 1:size(locationMatrix,2)
                    locationMatrix{i,j} = locationMatrix{i,j} + nConductors * (i-1);
                    regionMatrix{i,j}   = regionMatrix{i,j}   + nRegions * (i-1);
                end
            end
            
            %% Construct Domain Hull
          	nRegions = nRegions * nCopies;
            ang      = 2 * pi * fp;
            rot      = this.InitialAngle;
            rad      = [this.InnerRadius, this.OuterRadius];
          	dgOut    = Geometry2D.draw('Sector', 'Radius', rad, 'Angle', ang, 'Rotation', rot, 'PlotStyle', {'b'});
            this.DomainHull = copy(dgOut);
            if nRegions > 0
                dgOut = dgOut - [this.Regions.Geometry];
            end
            
            switch this.BackironType
                case BackironTypes.Laminated
                    dyna = DynamicsTypes.Static;
                case BackironTypes.Solid
                    dyna = DynamicsTypes.Grounded;
                otherwise
            end
            addModelRegion(this, [this.Name,'_DefaultRegion'], dgOut, this.DefaultMaterial, dyna);
            this.ModeledFraction = fp;
            
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %% Construct path sets
                    Ns = nConductors / turnsPerSlot * ones(1, Nb); %Ns strands per turn
                    Nt = turnsPerSlot * nSlots / Nb * ones(1, Nb); %Nt turns per bundle

                    [Pk, Ps] = this.makeDistributedWindingPathSet(locationMatrix, regionMatrix, Np, fp, Nb, Ns, Nt, Nc, sp);
                    
                    [Rk, Rs, Sk] = makeRegionAndStrandSets(Pk,Ps);
                    
                    this.Circuits.PathSets       = Pk;
                    this.Circuits.PathPolarity   = Ps;
                    this.Circuits.RegionSets     = Rk;
                    this.Circuits.RegionPolarity = Rs;
                    this.Circuits.StrandSets     = Sk;
                case CouplingTypes.Static
                    %% Construct turn sets
                    Nk   = turnsPerSlot / nConductors * ones(1, Nb); %Nk turns per conductor region
                    Nrb  = nConductors / Nb * ones(1, Nb);           %Ncb regions per bundle

                    [Tk,Ts,Tf] = this.makeDistributedWindingTurnSet(locationMatrix, regionMatrix, Np, fp, Nb, Nk, Nrb, Nc, sp);

                    this.Circuits.TurnSets     = Tk;
                    this.Circuits.TurnPolarity = Ts;
                    this.Circuits.TurnFactor   = Tf;
                otherwise
                    error('Unknown CouplingType %s', char(this.CouplingType));
            end
        end
        
      	function buildConcentratedWindingStator(this, fp)
            %% Check Configuration
          	nSlots       = this.Teeth;
            Np           = this.Poles;
            turnsPerSlot = this.Slot.Turns;
            nLayers      = this.Layers;
        	assert(mod(nSlots , 3) == 0,'MotorProto:StatorComponent', 'The number of teeth must be an integer multiple of 3. The current value is %d', nSlots);

            if nargin < 2
                if this.HasHalfWaveSymmetry
                    fp = 1 / this.SpatialSymmetries / 2;
                else
                    fp = 1 / this.SpatialSymmetries;
                end
            end
            
          	validateattributes(fp, {'numeric'}, {'scalar'});
            nCopies = this.GeometricSymmetries * fp;
            
          	assert(abs(nCopies-round(nCopies)) < sqrt(eps), 'MotorProto:RotatingMachineAssembly', 'ModelFraction must be an integer multiple of 1 / GeometricSymmetries');
            
            nCopies = round(nCopies);
            
            if nLayers == 1
                assert(mod(nCopies, 2) == 0)
            end
            
            assert(nCopies > 0, 'MotorProto:RotatingMachineAssembly', 'GeometricSymmetries * ModeledFraction >= 1 is required');
            
           	%% Construct Domain Hull
            ang = 2 * pi * fp;
            rot = this.InitialAngle;
            rad = [this.InnerRadius, this.OuterRadius];
          	dhull = Geometry2D.draw('Sector', 'Radius', rad, 'Angle', ang, 'Rotation', rot, 'PlotStyle', {'b'});
            this.DomainHull = dhull;
            
          	%% Build Slot
            [conductors, nonConductors, locationMatrix] = build(this.Slot, this.Name);
            
            nConductors   = numel(conductors{1});
            nSlotRegions  = nConductors + numel(nonConductors{1});
            nInputRegions = numel(this.InputRegions);
            
         	locationMatrix = repmat(locationMatrix, nCopies / (3-nLayers), 1);
            regionMatrix   = locationMatrix;
            
            da0 = pi / this.GeometricSymmetries + this.InitialAngle;
            if nLayers == 1
                dai = 4 * pi / this.GeometricSymmetries;
                daj = 2 * pi / this.GeometricSymmetries;
            else
                dai = 2 * pi / this.GeometricSymmetries;
                daj = 0;
            end
            
            Nc = 0;
            Nr = 0;
            for i = 1:(nCopies / (3-nLayers))
                for j = 1:2
                    slotRegions = copy([conductors{j}, nonConductors{j}]);
                    
                    angle = dai*(i-1) + daj*(j-1) + da0;
                    rotate(slotRegions, angle, [0,0]);
                    
                    [slotRegions.IsUserDefined] = deal(false);
                    addModelRegion(this, slotRegions);
                    
                    for k = 1:numel(locationMatrix{i,j})
                        locationMatrix{i,j}{k} = locationMatrix{i,j}{k} + Nc;
                        regionMatrix{i,j}{k}   = regionMatrix{i,j}{k} + Nr;
                    end
                    Nc = Nc + nConductors;
                    Nr = Nr + nSlotRegions;
                end
            end
            
            if nInputRegions > 0
                da = 2 * pi / this.GeometricSymmetries;
                for i = 1:nCopies
                    angle = da*(i-1) + da0;
                    inputRegions = copy(this.InputRegions);
                    rotate(inputRegions, angle, [0,0]);

                    [inputRegions.IsUserDefined] = deal(false);
                    addModelRegion(this, inputRegions);

                    Nr = Nr + nInputRegions;
                end
            end
            
            switch this.BackironType
                case BackironTypes.Laminated
                    dyna = DynamicsTypes.Static;
                case BackironTypes.Solid
                    dyna = DynamicsTypes.Grounded;
                otherwise
            end
            
            addModelRegion(this, [this.Name,'_DefaultRegion'], copy(dhull) - [this.Regions.Geometry], this.DefaultMaterial, dyna);
            this.ModeledFraction = fp;
            
            Nb = 3;
            switch this.CouplingType
                case CouplingTypes.Dynamic
                    %% Construct path sets
                    Ns = nConductors / turnsPerSlot * ones(1, Nb); %Ns strands per turn
                    Nt = turnsPerSlot * nSlots / Nb * ones(1, Nb); %Nt turns per bundle

                    [Pk, Ps] = this.makeConcentratedWindingPathSets(regionMatrix, fp, Nb, Ns, Nt);
                    [Rk, Rs, Sk] = makeRegionAndStrandSets(Pk,Ps);
                    
                    this.Circuits.PathSets       = Pk;
                    this.Circuits.PathPolarity   = Ps;
                    this.Circuits.RegionSets     = Rk;
                    this.Circuits.RegionPolarity = Rs;
                    this.Circuits.StrandSets     = Sk;
                case CouplingTypes.Static
                    if nLayers == 1
                        if nConductors == 1
                            Nk = turnsPerSlot * ones(1, Nb); %Nk turns per slot
                        else
                            Nk = turnsPerSlot / nConductors * ones(1, Nb);
                        end
                    else
                        if nConductors == 2
                            Nk = turnsPerSlot * ones(1, Nb); %Nk turns per slot
                        else
                            Nk = turnsPerSlot / nConductors * ones(1, Nb);
                        end
                    end
                    
                    [Tk,Ts,Tf] = this.makeConcentratedWindingTurnSet(locationMatrix, regionMatrix, Nb, Nk);

                    this.Circuits.TurnSets     = Tk;
                    this.Circuits.TurnPolarity = Ts;
                    this.Circuits.TurnFactor   = Tf;
                otherwise
                    error('Unknown CouplingType %s',char(this.CouplingType));
            end
        end
       
        %% Conductor Connectivity Functions
        function [Pk,Ps] = makeDistributedWindingPathSet(this, locationMatrix, regionMatrix, Np, fp, Nb, Ns, Nt, Nc, sp)
            Pk  = cell(1,Nb);
            Ps  = cell(1,Nb);
            Loc = cell(1,Nb);
            for i = 1:Nb
                Pk{i}  = cell(1,Ns(i));
                Ps{i}  = cell(1,Ns(i));
                Loc{i} = cell(1,Ns(i));
                for j = 1:Ns(i)
                    Pk{i}{j}  = zeros(1,Nt(i));
                    Ps{i}{j}  = zeros(1,Nt(i));
                    Loc{i}{j} = 1;
                end
            end
            
            %% Spoof location matrix for the entire stator
            nCopies = size(locationMatrix,1);
            locationMatrix = repmat(locationMatrix, 1/fp, 1);
            for k = 2:(1/fp)
                for i = ((k-1)*nCopies+1):(k*nCopies)
                    for j = 1:size(locationMatrix,2)
                        locationMatrix{i,j} = locationMatrix{i,j} + fp * Nc * (k-1);
                    end
                end
            end
            regionMatrix = repmat(regionMatrix, 1/fp, 1);
            
            %% Shift-strands in each pole-pair
            nSlots = size(regionMatrix,1);
            Nsbp   = nSlots / Np / Nb; %Number of slots per bundle per pole
            for i = 1:(Np/2)
                I = (i*Nsbp*2*Nb+1):nSlots;
                regionMatrix(I,:)   = circshift(regionMatrix(I,:),[0,1]);
                locationMatrix(I,:) = circshift(locationMatrix(I,:),[0,1]);
            end
            
            %% Place strands
            i = 1;
            s = 1;
            for m = 1:nSlots
                j = 1;
                
                for n = 1:size(locationMatrix,2)
                    if s == 1 %Ordering of strands depends on polarity due to winding
                        L = 1:1:length(locationMatrix{m,n});
                    else % s==-1
                        L = length(locationMatrix{m,n}):-1:1;
                    end

                    for l = L
                        k  = locationMatrix{m,n}(l);
                        ks = floor((k-1) / (fp*Nc));
                        
                        Pk{i}{j}(Loc{i}{j}) = regionMatrix{m,n}(l);
                        Ps{i}{j}(Loc{i}{j}) = s * (sp^ks);
                        
                        Loc{i}{j} = Loc{i}{j} + 1;
                        
                        if j == Ns(i)
                            j = 1;
                        else
                            j = j + 1;
                        end
                    end
                end
                
                if mod(m, Nsbp) == 0
                    s = -s;
                    i = i + 1;
                end
                
                if i > Nb
                    i = 1;
                end
            end
            
            %% Sort
            for i = 1:Nb
                for j = 1:Ns(i)
                    [Pk{i}{j},I] = sort(Pk{i}{j});
                    Ps{i}{j}     = Ps{i}{j}(I);
                end
            end
        end

        function [Pk,Ps] = makeConcentratedWindingPathSets(this, regionMatrix, fp, Nb, Ns, Nt)
          	Pk  = cell(1,Nb);
            Ps  = cell(1,Nb);
            Loc = cell(1,Nb);
            for i = 1:Nb
                Pk{i}  = cell(1,Ns(i));
                Ps{i}  = cell(1,Ns(i));
                Loc{i} = cell(1,Ns(i));
                for j = 1:Ns(i)
                    Pk{i}{j}  = zeros(1,fp*Nt(i));
                    Ps{i}{j}  = zeros(1,fp*Nt(i));
                    Loc{i}{j} = 1;
                end
            end
            
            wdg = generateConcentratedWindingLayout(this.Poles, this.Teeth, this.Layers);
            for i = 1:size(regionMatrix,1)
                for j = 1:size(regionMatrix,2)
                    l = 1;
                    k = abs(wdg(i,j));
                    s = sign(wdg(i,j));
                    
                    for m = 1:numel(regionMatrix{i,j})
                        for n = 1:numel(regionMatrix{i,j}{m})
                            Pk{k}{l}(Loc{k}{l}) = regionMatrix{i,j}{m}(n);
                            Ps{k}{l}(Loc{k}{l}) = s;
                            
                            Loc{k}{l} = Loc{k}{l} + 1;
                            if l == Ns(k)
                                l = 1;
                            else
                                l = l + 1;
                            end
                        end
                    end
                end
            end
            
            for i = 1:Nb
                for j = 1:Ns
                    Pk{i}{j} = repmat(Pk{i}{j},1,1/fp);
                    Ps{i}{j} = repmat(Ps{i}{j},1,1/fp);
                    
                    [Pk{i}{j},I] = sort(Pk{i}{j});
                    Ps{i}{j}     = Ps{i}{j}(I);
                end
            end
        end
        
        function [Tk,Ts,Tf] = makeDistributedWindingTurnSet(this, locationMatrix, regionMatrix, Np, fp, Nb, Nk, Nrb, Nc, sp)
            Tk = cell(1, Nb);
            Ts = cell(1, Nb);
            Tf = cell(1, Nb);
            Loc = ones(1, Nb);
            for i = 1:Nb
                if Nrb(i) > 1
                    Tk{i} = zeros(1, Nrb(i));
                    Ts{i} = zeros(1, Nrb(i));
                    Tf{i} = zeros(1, Nrb(i));
                else
                    Tk{i} = zeros(1, 1);
                    Ts{i} = zeros(1, 1);
                    Tf{i} = zeros(1, 1);
                end
            end
            
            i = 1;
            s = 1;
            nSlots = size(locationMatrix,1);
            Nsbp   = nSlots / (fp * Nb * Np); %Number of slots per bundle per pole
            for m = 1:nSlots
                for n = 1:size(locationMatrix,2)
                    for l = 1:numel(locationMatrix{m,n})
                        k  = locationMatrix{m,n}(l);
                        ks = floor((k-1) / (fp*Nc));
                        
                        Tk{i}(Loc(i)) = regionMatrix{m,n}(l);
                        Ts{i}(Loc(i)) = s * (sp^ks);
                        Tf{i}(Loc(i)) = Nk(i);
                        
                        Loc(i)        = Loc(i) + 1;
                    end
                end
                
                if mod(m, Nsbp) == 0
                    s = -s;
                    i = i + 1;
                end
                
                if i > Nb
                    i = 1;
                end
            end
            
            %% Sort
            for i = 1:Nb
            	[Tk{i},I] = sort(Tk{i});
               	Ts{i}     = Ts{i}(I);
               	Tf{i}     = Tf{i}(I);
            end
        end
        
        function [Tk,Ts,Tf] = makeConcentratedWindingTurnSet(this, locationMatrix, regionMatrix, Nb, Nk)
            Tk = cell(1, Nb);
            Ts = cell(1, Nb);
            Tf = cell(1, Nb);
            Loc = ones(1, Nb);
            for i = 1:Nb
                Tk{i} = zeros(1, 1);
                Ts{i} = zeros(1, 1);
                Tf{i} = zeros(1, 1);
            end
            
            wdg = generateConcentratedWindingLayout(this.Poles, this.Teeth, this.Layers);
            
            [M,N] = size(locationMatrix);
            for i = 1:M
                for j = 1:N
                    k = abs(wdg(i,j));
                    s = sign(wdg(i,j));
                    
                    for m = 1:numel(regionMatrix{i,j})
                        for n = 1:numel(regionMatrix{i,j}{m})
                            Tk{k}(Loc(k)) = regionMatrix{i,j}{m}(n);
                            Ts{k}(Loc(k)) = s;
                            Tf{k}(Loc(k)) = Nk(k);

                            Loc(k) = Loc(k) + 1;
                        end
                    end
                end
            end
            
            %% Sort
            for i = 1:Nb
            	[Tk{i},I] = sort(Tk{i});
               	Ts{i}     = Ts{i}(I);
               	Tf{i}     = Tf{i}(I);
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
            
            if ~isempty(this.Slot.Shape)
                domainHull = domainHull - this.Slot.Shape;
                conductors = build(this.Slot, this.Name);  
                conductors = [conductors.Geometry];
            else
                conductors = [];
            end
            
            plot([domainHull, conductors, regionGeometry]);
        end
    end
    
    methods (Static)
        function assemblyOut = newAssembly(varargin)
            assemblyOut = Stator(varargin{:});
        end
    end
end