classdef MatrixFactory
    properties (SetAccess = protected, Dependent)
        Mesh
        Assemblies
    end
    
    properties (SetAccess = protected)
        Model
        
        %% Matrix components are stored in structures for fast access
        %   C(t)*x_t + K(t)*x + g(x,t)   = F*f(t), Continuous Time Equation
        %   C(t)*y_t + K(t)*y + G(x,t)*y = - r(t), Linearized Equation
        
        Del         %Discrete del operator matrices, div, grad, curl
        Coupling    %Matrices related to external circuit coupling
        Boundary    %Matrices for performing boundary operations
        
        Mass        %C(t)
        Stiffness   %K(t)
        Jacobian    %g(x,t), G(x,t)
        Exogenous   %F

        PostProcessing %Matrices for solution post processing
        
        Index %Local and global indices of unknowns
        
        DomainPartitions %Domain partition information
    end
    
    methods %Accessors
        %% Constructor
        function this = MatrixFactory(model)
            if nargin > 0
                this.Model = model;
                this = build(this);
            end
        end
        
        %% Getters
        function value = get.Mesh(this)
            value = this.Model.Mesh;
        end
        
        function value = get.Assemblies(this)
            value = [this.Mesh.Assembly];
        end
        
        %% Preprocessing functions
        function this = build(this)
            this = buildIndexVectors(this);
            this = partitionDomains(this);
            
            this = buildDelMatrices(this);
            this = buildCouplingPrimitives(this);
            
            this = buildPostProcessingMatrices(this);
            
            this = buildBoundaryMatrices(this);
            
            this = buildMassMatrices(this);
            this = buildStiffnessMatrices(this);
            this = buildJacobianMatrices(this);
            this = buildExogenousMatrices(this);
            
            this = buildCircuitMatrices(this);
            
            this = applyBoundaryConditions(this);
        end
    end
    
    methods %Matrix Construction
        function this  = buildIndexVectors(this)
            this.Index = buildMatrices(this, this.Index, 'buildLocalIndexVectors');
            this.Index = buildGlobalIndexVectors(this, this.Index);
        end
        
        function index = buildGlobalIndexVectors(this,index)
            index.Global(1)   = index.Local(1);
            index.Local(1).X  = 1:index.Local(1).Unknowns;
            index.Global(1).X = 1:index.Global(1).Unknowns;
            nUnknowns         = index.Local(1).Unknowns;
            
            for i = 2:numel(index.Local);
             	index.Global(i)   = buildGlobalIndexField(this, index.Local(i), nUnknowns);
                index.Local(i).X  = 1:index.Local(i).Unknowns;
                index.Global(i).X = (nUnknowns + 1):index.Global(i).Unknowns;
                nUnknowns         = index.Global(i).Unknowns;
            end
        end
        
        function structure = buildGlobalIndexField(this, structure, nUnknowns)
            nStructs   = numel(structure);
            fieldNames = fields(structure);
            
            for i = 1:nStructs
                for j = 1:numel(fieldNames)
                    if isnumeric(structure(i).(fieldNames{j}))
                        structure(i).(fieldNames{j}) = structure(i).(fieldNames{j}) + nUnknowns;
                    elseif iscell(structure(i).(fieldNames{j}))
                        n = numel(structure(i).(fieldNames{j}));
                        for k = 1:n
                            structure(i).(fieldNames{j}){k} = structure(i).(fieldNames{j}){k} + nUnknowns;
                        end
                    else
                        structure(i).(fieldNames{j})  = buildGlobalIndexField(this, structure(i).(fieldNames{j}), nUnknowns);
                    end
                end
            end
        end
        
        function [structure, newFields, pbcType] = buildLocalIndexVectors(this, structure, iMesh)
            newFields = {'Local'};
            pbcType   = {'None'};
            
            if isfield(structure,'Local')
                local = structure.Local;
            else
                local = struct([]);
            end
            
            nUnknowns = 0;
            
            %% Magnetic Vector Potential
            mesh           = this.Mesh(iMesh);
            nNodes         = numel(mesh.X);
            local(iMesh).A = 1:nNodes;
            
            nUnknowns = nUnknowns + nNodes;
            
            %% Floating Regions (conducting regions where i = 0, e.g. permanent magnets)
            assembly   = this.Assemblies(iMesh);
         	regions    = assembly.Regions;
            nRegions   = numel(regions);
            dynamics   = [regions.Dynamics];
            isFloating = (dynamics == DynamicsTypes.Floating);
            nFloating  = sum(isFloating);
            
            local(iMesh).Regions             = NaN(1, nRegions);
            local(iMesh).Regions(isFloating) = (1:nFloating) + nUnknowns;
            
            nUnknowns = nUnknowns + nFloating;
            
            %% Circuits
           	circuits  = assembly.Circuits;
            nCircuits = numel(circuits);
            
            if nCircuits > 0
                nCircuitUnknowns      = circuits.getSize;
                local(iMesh).Circuits = (1:nCircuitUnknowns) + nUnknowns;
                circuits.Index        = local(iMesh).Circuits;
                nUnknowns             = nUnknowns + nCircuitUnknowns;
            else
                local(iMesh).Circuits = [];
            end
            
            %% Boundary Conditions
            local(iMesh).Boundary.Tangent(1).Nodes = mesh.PeriodicBoundaryNodes(1,:);
            local(iMesh).Boundary.Tangent(2).Nodes = mesh.PeriodicBoundaryNodes(1,:);
            
            local(iMesh).Boundary.Radius(1).Nodes  = unique(mesh.RadialBoundaryEdges{1});
            local(iMesh).Boundary.Radius(2).Nodes  = unique(mesh.RadialBoundaryEdges{2});
            
            local(iMesh).Unknowns = nUnknowns;
            structure.Local = local;
        end
        
        function this = partitionDomains(this)
            mesh            = this.Mesh;
            nMesh           = length(mesh);
            domainPartition = struct([]);
            for iMesh = 1:nMesh
                nElements                                 = length(mesh(iMesh).Elements);
                domainPartition(iMesh).Domains            = 1;
                domainPartition(iMesh).Domain(1).Elements = true(1,nElements);
            end
            this.DomainPartitions = domainPartition;
        end
        
        %% Del Matrices
        function this = buildDelMatrices(this)
            this.Del = buildMatrices(this, this.Del, 'buildCurlMatrices');
        end
        
        function [structure, newFields, pbcType] = buildCurlMatrices(this, structure, iMesh)
            %% New Field Definitions
            newFields = {'Curl'};
            pbcType   = {'None'};
            
            %% Build Matrices
            mesh      = this.Mesh(iMesh);
           	el        = mesh.Elements;
            elArea    = mesh.ElementAreas;
            nElements = length(el);
            nUnknowns = this.Index.Local(iMesh).Unknowns;
            
        	x = mesh.X(el);
            y = mesh.Y(el);
            
            b =  [y(2,:)-y(3,:); y(3,:)-y(1,:); y(1,:)-y(2,:)];
            c = -[x(2,:)-x(3,:); x(3,:)-x(1,:); x(1,:)-x(2,:)];
            
            i = [1:nElements,1:nElements,1:nElements];
            j = [el(1,:),el(2,:),el(3,:)];
            
            s = 1./elArea;
            A = sparse(1:nElements,1:nElements,s,nElements,nElements);
            
            %% Make node to element and element to node curl matrices
            s = [b(1,:), b(2,:), b(3,:)] / 2;
            structure.Curl(iMesh).Ye2Zn = sparse(j,i,s,nUnknowns,nElements);
            structure.Curl(iMesh).Zn2Ye = -A * structure.Curl(iMesh).Ye2Zn';
            
            s = -[c(1,:), c(2,:), c(3,:)] / 2;
            structure.Curl(iMesh).Xe2Zn = sparse(j,i,s,nUnknowns,nElements);
            structure.Curl(iMesh).Zn2Xe = -A * structure.Curl(iMesh).Xe2Zn';
        end
        
        %% Coupling matrices
        function this = buildCouplingPrimitives(this)
            mesh  = this.Mesh;
            nMesh = numel(mesh);
            coupling = struct('Integral', [], 'Area', [], 'Conductivity', [], 'ModeledFraction', [], 'Length', [], 'Region2Element', 'Node2Element');
            
            for i = 1:nMesh
                index     = this.Index.Local(i);
                nUnknowns = index.Unknowns;
                assembly  = mesh(i).Assembly;
                regions   = [assembly.Regions];
            
                nRegions = numel(regions);
                inte     = cell(1, nRegions);
                area     = zeros(1, nRegions);
                cond     = zeros(1, nRegions);
                r2el     = cell(1, nRegions);
                n2el     = cell(3, nRegions);
                
                els       = mesh(i).Elements;
                elRegions = mesh(i).ElementRegions;
                elAreas   = mesh(i).ElementAreas;
                nElements = numel(elRegions);
                for j = 1:nRegions
                    J = (elRegions == j);
                    K = find(J);
                    n = sum(J);
                    
                    rows = ones(1, 3*n);
                    cols = [els(1, J), els(2, J), els(3, J)];
                    vals = [elAreas(J), elAreas(J), elAreas(J)] / 3;

                    inte{j} = sparse(rows, cols, vals, 1, nUnknowns);
                    area(j) = sum(elAreas(J));
                    cond(j) = regions(j).Material.sigma;
                    r2el{j} = sparse(K, ones(1, n), ones(1, n), nElements, 1);
                    
                    for k = 1:3
                        n2el{k,j} = sparse(K, els(k, J), ones(1, n), nElements, nUnknowns);
                    end
                end
                
                coupling(i).Integral        = inte;
                coupling(i).Area            = area;
                coupling(i).Conductivity    = cond;
                coupling(i).Region2Element  = r2el;
                coupling(i).Node2Element    = n2el;
                coupling(i).ModeledFraction = assembly.ModeledFraction;
                coupling(i).Length          = assembly.Length;
                coupling(i).Elements        = nElements;
                coupling(i).Unknowns        = index.Unknowns;
            end
            
            this.Coupling = coupling;
        end
        
        %% Post Processing Matrices
        function this = buildPostProcessingMatrices(this)
            mesh           = this.Mesh;
            index          = this.Index;
            coupling       = this.Coupling;
            nMesh          = numel(mesh);
            postProcessing = struct.empty(0,nMesh);
            
            for i = 1:nMesh
                assembly  = mesh(i).Assembly;
                circuits  = assembly.Circuits;
                nCircuits = length(circuits);
                nUnknowns = index.Local(i).Unknowns;
                I         = speye(nUnknowns,nUnknowns);
                
              	postProcessing(i).Full2Reduced = this.applyPeriodicBoundaryConditions(I, i, 'ReduceRow');
                postProcessing(i).Reduced2Full = this.applyPeriodicBoundaryConditions(I, i, 'RestoreRow');
                
                %% Solution to Magnetic Vector Potential
                postProcessing(i).X2A = I(index.Local(i).A,:);
                
                %% Circuit contributions
                if nCircuits > 0
                    %% Bundle currents
                    postProcessing(i).F2I   = circuits.F2I(coupling(i));
                    postProcessing(i).X2I   = circuits.X2I(coupling(i));
                    postProcessing(i).X_t2I = circuits.X_t2I(coupling(i));
                    
                    %% Bundle voltages
                    postProcessing(i).F2V   = circuits.F2V(coupling(i));
                    postProcessing(i).X2V   = circuits.X2V(coupling(i));
                    postProcessing(i).X_t2V = circuits.X_t2V(coupling(i));
                    
                    %% Bundle flux linkages
                    postProcessing(i).F2Lambda   = circuits.F2Lambda(coupling(i));
                    postProcessing(i).X2Lambda   = circuits.X2Lambda(coupling(i));
                    postProcessing(i).X_t2Lambda = circuits.X_t2Lambda(coupling(i));
                    
                    %% Electric fields
                    postProcessing(i).F2E   = circuits.F2E(coupling(i));
                    postProcessing(i).X2E   = circuits.X2E(coupling(i));
                    postProcessing(i).X_t2E = circuits.X_t2E(coupling(i));
                    
                    %% Current density
                    postProcessing(i).F2J   = circuits.F2J(coupling(i));
                    postProcessing(i).X2J   = circuits.X2J(coupling(i));
                    postProcessing(i).X_t2J = circuits.X_t2J(coupling(i));
                else
                	%% Bundle currents
                    postProcessing(i).X2I   = sparse(0,nUnknowns);
                    postProcessing(i).X_t2I = sparse(0,nUnknowns);
                    
                    %% Bundle voltages
                    postProcessing(i).X2V   = sparse(0,nUnknowns);
                    postProcessing(i).X_t2V = sparse(0,nUnknowns);
                    
                    %% Bundle flux linkages
                    postProcessing(i).X2Lambda   = sparse(0,nUnknowns);
                    postProcessing(i).X_t2Lambda = sparse(0,nUnknowns);
                    
                    %% Electric Field
                    n = numel(mesh(i).ElementRegions);
                    postProcessing(i).F2E = sparse(n, 0);
                    postProcessing(i).F2J = sparse(n, 0);
                    
                    postProcessing(i).X2E = sparse(n, nUnknowns);
                    postProcessing(i).X2J = sparse(n, nUnknowns);
                    
                    %% Current Density
                    postProcessing(i).X_t2E = cell(1,3);
                    postProcessing(i).X_t2J = cell(1,3);
                    
                    for j = 1:3
                        postProcessing(i).X_t2E{j} = sparse(n, nUnknowns);
                        postProcessing(i).X_t2J{j} = sparse(n, nUnknowns);
                    end
                end
                
                %% Floating regions
                regions  = [assembly.Regions];
                nRegions = numel(regions);
                dynamics = [regions.Dynamics];
                for j = 1:nRegions
                    k = index.Local(i).Regions(j);
                    s = coupling(i).Conductivity(j);
                    
                    if dynamics(j) == DynamicsTypes.Floating
                        postProcessing(i).X2E(:,k) = -coupling(i).Region2Element{j};
                        postProcessing(i).X2J(:,k) = -s * coupling(i).Region2Element{j};
                    end
                    
                    if (dynamics(j) == DynamicsTypes.Floating) || (dynamics(j) == DynamicsTypes.Grounded)
                        for l = 1:3
                            postProcessing(i).X_t2E{l} = postProcessing(i).X_t2E{l} - coupling(i).Node2Element{l,j};
                            postProcessing(i).X_t2J{l} = postProcessing(i).X_t2J{l} - s * coupling(i).Node2Element{l,j};
                        end
                    end
                end
                
                %% Sobolev Norm Weighting Matrices
                el	      = mesh(i).Elements;
                elArea    = mesh(i).ElementAreas;
                nElements = numel(elArea);
                
                ii = [el(1,:), el(2,:), el(3,:),...
                      el(1,:), el(2,:), el(1,:),...
                      el(2,:), el(3,:), el(3,:)];
                
                jj = [el(1,:), el(2,:), el(3,:),...
                      el(2,:), el(3,:), el(3,:),...
                      el(1,:), el(2,:), el(1,:)];
                
                ss = elArea / 12;
                ss = [repmat(2*ss,1,3), repmat(ss,1,6)];
                postProcessing(i).SobolevA = sparse(ii,jj,ss,nUnknowns,nUnknowns);
                
                ii = 1:nElements;
                jj = 1:nElements;
                ss = elArea;
                postProcessing(i).SobolevB = sparse(ii,jj,ss,nElements,nElements);
            end
            this.PostProcessing = postProcessing;
        end
        
        %% Boundary Matrices
        function this = buildBoundaryMatrices(this)
            this.Boundary.Radial = buildMatrices(this, struct(''), 'buildRadialBoundaryMatrices');
        end
        
        function [structure, newFields, pbcType] = buildRadialBoundaryMatrices(this, structure, iMesh)
            %% New Field Definitions
            newFields = {'R'   , 'D'                , 'S'  , 'P'                       , 'F'   ,'G'};
            pbcType   = {'None', 'BoundaryOperation', 'Row', 'InverseBoundaryOperation', 'None', 'None'};
            
            if isempty(structure)
                structure = cell2struct(cell(1, numel(newFields)), newFields, 2);
            end
            
            %% Get boundary radii
            meshes    = this.Mesh;
            nMeshes   = numel(meshes);
            r         = [meshes.RadialBoundaryRadii];
            rSelf     = r((2*iMesh-1):(2*iMesh));
            rOpposite = [0 inf];
            
            if iMesh > 1
                rOpposite(1) = r(2*iMesh - 2);
            end
            
            if iMesh < nMeshes
                rOpposite(2) = r(2*iMesh + 1);
            end
            
            %% get boundary edges
            mesh       = meshes(iMesh);
            bEdges     = mesh.RadialBoundaryEdges;
            
            %% get angular velocity
            assembly   = mesh.Assembly;
            bVelocity  = assembly.AngularVelocity;
            bHarmonics = this.getRadialBoundaryHarmonics(iMesh);
            
            %% calculate matrices
            nUnknowns  = this.Index.Local(iMesh).Unknowns;
            for i = 1:2
                n          = bHarmonics{i}.';
                nHarmonics = numel(n);
            
                if rOpposite(i) == 0 || isinf(rOpposite(i))
                    R = sparse(1, 1, 1);
                else
                    R = bVelocity(1) * n;
                end
            
                x = mesh.X(bEdges{i});
                y = mesh.Y(bEdges{i});
                t = atan2(y, x);

                [t,K] = sort(t);
                b     = bEdges{i};
                for k = 1:length(K)
                    b(:, k) = b(K(:,k), k);
                end
            
                %% "dft" matrix
                s1 =  1i * bsxfun(@rdivide, exp(-1i * n * t(2,:)), n) + (exp(-1i * n * t(1,:)) - exp(-1i * n * t(2,:))) ./ (n.^2 * (t(1,:) - t(2,:)));
                s2 = -1i * bsxfun(@rdivide, exp(-1i * n * t(1,:)), n) + (exp(-1i * n * t(2,:)) - exp(-1i * n * t(1,:))) ./ (n.^2 * (t(1,:) - t(2,:)));
                
                isZero = (n==0);
                if any(isZero)
                    s1(isZero,:) = (t(2,:)-t(1,:))/2;
                    s2(isZero,:) = (t(2,:)-t(1,:))/2;
                end
                
                s  = [ reshape(s1, 1, []), reshape(s2, 1, [])] / 2 / pi;            
                k  = [ reshape((1:nHarmonics).' * ones(size(b(2, :))),1,[]), reshape((1:nHarmonics).' * ones(size(b(1,:))), 1, [])];
                    
                l  = [ reshape(ones(size(n))*b(2,:), 1, []), reshape(ones(size(n))*b(1,:), 1, [])];
            
            	D = sparse(k,l,s,nHarmonics,nUnknowns);
                S = (-1)^(i) * D' * 2 * pi * rSelf(i);
                
                %% "idft" matrix
                I      = any(D~=0);
                p      = D(:,I);
                p      = (p'*p)\p';
                P      = D';
                P(I,:) = p;
                
                %% boundary transfer matrices
                if isinf(rOpposite(i))
                    sf         = abs(n) / rSelf(i);
                    sf(isZero) = mu_o;
                    sg         = zeros(1,nHarmonics);
                elseif rOpposite(i) == 0;
                    sf         = -abs(n) / rSelf(i);
                    sf(isZero) = mu_o;
                    sg         = zeros(1,nHarmonics);
                else
                    a  = rOpposite(i) / rSelf(i);
                    
                    sf = (n     .* (a.^n + a.^(-n))) ./ (rSelf(i)     * (a.^n - a.^(-n)));
                    sg = (2 * n                    ) ./ (rOpposite(i) * (a.^n - a.^(-n)));
                    
                    sf(isZero) = (log(a)*rSelf(i)    ).^(-1);
                    sg(isZero) = (log(a)*rOpposite(i)).^(-1);
                end
                F = sparse(1:nHarmonics, 1:nHarmonics, sf / mu_o, nHarmonics, nHarmonics);
                G = sparse(1:nHarmonics, 1:nHarmonics, sg / mu_o, nHarmonics, nHarmonics);
                
                if i == 1
                    structure.R(iMesh).Inner = R;
                    structure.D(iMesh).Inner = D;
                    structure.S(iMesh).Inner = S;
                    structure.P(iMesh).Inner = P;
                    structure.F(iMesh).Inner = F;
                    structure.G(iMesh).Inner = G;
                else
                    structure.R(iMesh).Outer = R;
                    structure.D(iMesh).Outer = D;
                    structure.S(iMesh).Outer = S;
                    structure.P(iMesh).Outer = P;
                    structure.F(iMesh).Outer = F;
                    structure.G(iMesh).Outer = G;
                end
            end
        end
        
        %% Stiffness Matrices
        function this = buildStiffnessMatrices(this)
            stiffness      = struct('Reluctivity', []);
            this.Stiffness = this.buildReluctivityMatrices(stiffness);
        end
        
      	function structure = buildReluctivityMatrices(this, structure)
            reluctivity = struct([]);
            
            for iMesh = numel(this.Mesh):-1:1
                mesh      = this.Mesh(iMesh);
                regions   = mesh.Regions;
                nRegions  = length(regions);
                mu        = mu_o*ones(nRegions,1);
                for iRegion = 1:nRegions
                    if regions(iRegion).Material.Linear
                        mu(iRegion) = regions(iRegion).Material.Permeability;
                    end
                end
                mu        = mu(mesh.ElementRegions,:);
                el        = mesh.Elements.';
                x         = mesh.X(el);
                y         = mesh.Y(el);
                index     = this.Index.Local(iMesh);
                nUnknowns = index.Unknowns;
                dp        = this.DomainPartitions(iMesh);
                nDomains  = dp.Domains;

                Kff = cell(1,nDomains);
                for iDomain = 1:nDomains
                    I = dp.Domain(iDomain).Elements;

                    dLdX = -[y(I,2) - y(I,3), y(I,3) - y(I,1), y(I,1) - y(I,2)];
                    dLdY = -[x(I,2) - x(I,3), x(I,3) - x(I,1), x(I,1) - x(I,2)];
                    den  =  (2*(x(I,1).*y(I,2) - x(I,1).*y(I,3) - y(I,1).*x(I,2) + y(I,1).*x(I,3) + x(I,2).*y(I,3) - y(I,2).*x(I,3)).*mu(I));

                    val = [ dLdX(:,1).*dLdX(:,1) + dLdY(:,1).*dLdY(:,1),...
                            dLdX(:,1).*dLdX(:,2) + dLdY(:,1).*dLdY(:,2),...
                            dLdX(:,1).*dLdX(:,3) + dLdY(:,1).*dLdY(:,3),...
                            dLdX(:,2).*dLdX(:,1) + dLdY(:,2).*dLdY(:,1),...
                            dLdX(:,2).*dLdX(:,2) + dLdY(:,2).*dLdY(:,2),...
                            dLdX(:,2).*dLdX(:,3) + dLdY(:,2).*dLdY(:,3),...
                            dLdX(:,3).*dLdX(:,1) + dLdY(:,3).*dLdY(:,1),...
                            dLdX(:,3).*dLdX(:,2) + dLdY(:,3).*dLdY(:,2),...
                            dLdX(:,3).*dLdX(:,3) + dLdY(:,3).*dLdY(:,3)];

                    val = bsxfun(@rdivide, val, den);

                    row = [ el(I,1), el(I,1), el(I,1),...
                            el(I,2), el(I,2), el(I,2),...
                            el(I,3), el(I,3), el(I,3)];

                    col = [ el(I,1), el(I,2), el(I,3),...
                            el(I,1), el(I,2), el(I,3),...
                            el(I,1), el(I,2), el(I,3)];

                    Kff{iDomain} = sparse(row, col, val, nUnknowns, nUnknowns);
                end

                if nDomains == 1
                    reluctivity(iMesh).Kff = Kff{1};
                else
                    reluctivity(iMesh).Kff = Kff;
                end

                reluctivity(iMesh).Kfc = sparse(nUnknowns, nUnknowns);
                reluctivity(iMesh).Kcf = sparse(nUnknowns, nUnknowns);
                reluctivity(iMesh).Kcc = sparse(nUnknowns, nUnknowns);

                %% Floating Regions
                dynamics = [regions.Dynamics];
                coupling = this.Coupling(iMesh);
                index    = this.Index.Local(iMesh);
                for i = 1:nRegions
                    if dynamics(i) == DynamicsTypes.Floating
                        [cols, rows, vals] = find(coupling.Integral{i});
                        s    = coupling.Conductivity(i);
                        vals = s * vals;

                        j    = index.Regions(i);
                        cols = j * cols;

                        reluctivity(iMesh).Kfc = reluctivity(iMesh).Kfc + sparse(rows, cols, vals, nUnknowns, nUnknowns);

                        reluctivity(iMesh).Kcc(j,j) = reluctivity(iMesh).Kcc(j,j) + s * coupling.Area(i);
                    end
                end
                
                %% Apply boundary conditions
                reluctivity = applyPeriodicBoundaryConditions(this, reluctivity, iMesh, 'both');
            end
            
            %% Append to Structure
            structure.Reluctivity = reluctivity;
        end
        
        %% Mass Matrices
        function this = buildMassMatrices(this)
            mass      = struct('Conducitivity', []);
        	this.Mass = this.buildConductivityMatrices(mass);
        end
        
        function structure = buildConductivityMatrices(this, structure)
            conductivity = struct([]);
            
            for iMesh = numel(this.Mesh):-1:1
                mesh      = this.Mesh(iMesh);
                regions   = mesh.Regions;
                nRegions  = numel(regions);
                elRegions = mesh.ElementRegions;
                dynamics  = [regions.Dynamics];
                materials = [regions.Material];
                elCond    = [materials.sigma];

                isStatic  = (dynamics == DynamicsTypes.Static);
                isDynamic = ismember(elRegions, find(~isStatic));

                dp        = this.DomainPartitions(iMesh);
                nDomains  = dp.Domains;

                index     = this.Index.Local(iMesh);
                nUnknowns = index.Unknowns;

                %% Finite Element Mass Matrix
                Cff = cell(1,nDomains);
                for iDomain = 1:nDomains
                    I         = isDynamic & dp.Domain(iDomain).Elements;
                    elRegions = elRegions(I);
                    el        = mesh.Elements(:,I);
                    elArea    = mesh.ElementAreas(I);
                    elCond    = elCond(elRegions);

                    i = [el(1,:), el(2,:), el(3,:),...
                         el(1,:), el(2,:), el(1,:),...
                         el(2,:), el(3,:), el(3,:)];

                    j = [el(1,:), el(2,:), el(3,:),...
                         el(2,:), el(3,:), el(3,:),...
                         el(1,:), el(2,:), el(1,:)];

                    s = elCond .* elArea / 12;
                    s = [repmat(2*s,1,3), repmat(s,1,6)];

                    Cff{iDomain} = sparse(i,j,s,nUnknowns,nUnknowns);
                end

                if nDomains == 1
                    conductivity(iMesh).Cff = Cff{1};
                else
                    conductivity(iMesh).Cff = Cff;
                end
                
                conductivity(iMesh).Cfc = sparse(nUnknowns, nUnknowns);
                conductivity(iMesh).Ccf = sparse(nUnknowns, nUnknowns);
                conductivity(iMesh).Ccc = sparse(nUnknowns, nUnknowns);

                %% Floating Regions
                coupling = this.Coupling(iMesh);
                index    = this.Index.Local(iMesh);
                for i = 1:nRegions
                    if dynamics(i) == DynamicsTypes.Floating
                        [rows, cols, vals] = find(coupling.Integral{i});

                        vals = coupling.Conductivity(i) * vals;

                        j    = index.Regions(i);
                        rows = j * rows;

                        conductivity(iMesh).Ccf = conductivity(iMesh).Ccf + sparse(rows, cols, vals, nUnknowns, nUnknowns);
                    end
                end

                %% Apply boundary conditions
                conductivity = applyPeriodicBoundaryConditions(this, conductivity, iMesh, 'both');
            end
            
            %% Append to Structure
            structure.Conductivity = conductivity;
        end
        
        %% Jacobian Matrices
        function this = buildJacobianMatrices(this)
            this.Jacobian = buildMatrices(this, this.Jacobian, 'buildMagnetizationJacobianMatrices');
            this.Jacobian = buildMatrices(this, this.Jacobian, 'buildFluxDensityJacobianMatrices'  );
        end
        
        function [structure, newFields, pbcType] = buildMagnetizationJacobianMatrices(this, structure, iMesh)
            %% New Field Definitions
            newFields = {'MagnetizationCurrent'};
            pbcType  = {'Row'};
            
            if isfield(structure, newFields{1})
                magnetizationCurrent = structure.MagnetizationCurrent;
            else
                magnetizationCurrent = struct([]);
            end
            
            %% Build Matrices
            curl = this.Del.Curl(iMesh);
            
            magnetizationCurrent(iMesh).dIzdMx = curl.Xe2Zn;
            magnetizationCurrent(iMesh).dIzdMy = curl.Ye2Zn;
            
            %% Update Structure
            structure.MagnetizationCurrent = magnetizationCurrent;
        end
        
        function [structure, newFields, pbcType] = buildFluxDensityJacobianMatrices(this, structure, iMesh)
            %% Neew Field Definitions
            newFields = {'FluxDensity'};
            pbcType   = {'Column'};
            
            if isfield(structure, newFields{1})
                fluxDensity = structure.FluxDensity;
            else
                fluxDensity = struct([]);
            end
            
            %% Build Matrices
            curl = this.Del.Curl(iMesh);
            
            fluxDensity(iMesh).dBxdXz = curl.Zn2Xe;
            fluxDensity(iMesh).dBydXz = curl.Zn2Ye;
            
            %% Update Structure
            structure.FluxDensity = fluxDensity;
        end
        
        %% Exogenous Input Matrices
        function this = buildExogenousMatrices(this)
            this.Exogenous = buildMatrices(this, this.Exogenous, 'buildMagneticInputMatrices');
        end
        
        function [structure, newFields, pbcType] = buildMagneticInputMatrices(this, structure, iMesh)
            %% New Field Definitions
            newFields = {'Magnetic'};
            pbcType   = {'Row'};
            
            if isfield(structure, newFields{1})
                mqsInput = structure.Magnetic;
            else
                mqsInput = struct([]);
            end
            
            assembly  = this.Assemblies(iMesh);
            circuits  = assembly.Circuits;
            nCircuits = numel(circuits);
            index     = this.Index.Local(iMesh);
            nUnknowns = index.Unknowns;
            coupling  = this.Coupling(iMesh);
            
            %% External Circuit Coupling
            if nCircuits > 0
                mqsInput(iMesh).Ff = circuits.Ff(coupling);
                mqsInput(iMesh).Fc = circuits.Fc(coupling);
            else
                mqsInput(iMesh).Ff = sparse(nUnknowns, 0);
                mqsInput(iMesh).Fc = sparse(nUnknowns, 0);
            end
            
            %% UpdateStructure
            structure.Magnetic = mqsInput;
        end
        
        %% Circuit Matrices
        function this = buildCircuitMatrices(this)
            assemblies  = this.Assemblies;
            nAssemblies = numel(assemblies);
            bcFuns      = this.getBCFuns;
            coupling    = this.Coupling;
            for i = 1:nAssemblies
                circuit = assemblies(i).Circuits;
                if numel(circuit) > 0
                	circuit.build(coupling(i), bcFuns{i});
                end
            end
        end
        
        %% Boundary Conditions
        function this = applyBoundaryConditions(this)
            this = applyTangentialBoundaryConditions(this);
            this = applyRadialBoundaryConditions(this);
        end
        
        function bcFuns = getBCFuns(this)
            if this.Model.HasHalfWaveSymmetry
                sgn = -1;
            else
                sgn = 1;
            end
            
            nMesh  = numel(this.Mesh);
            bcFuns = cell(1,nMesh);
            
            for i = 1:nMesh
                mesh = this.Mesh(i);
                mf   = mesh.Assembly.ModeledFraction;
                I    = mesh.PeriodicBoundaryNodes(1, :);
                J    = mesh.PeriodicBoundaryNodes(2, :);
                N    = numel(mesh.X);
                K    = setdiff(1:N, [I, J]);
                
                bcFuns{i} = @(mat,action)(MatrixFactory.pbcSubroutine(mat,action,I,J,K,sgn,mf));
            end
        end
        
        function mats = applyPeriodicBoundaryConditions(this, mats, iMesh, action)
            if ~strcmpi(action,'none')
                if this.Model.HasHalfWaveSymmetry
                    sgn = -1;
                else
                    sgn = 1;
                end

                mesh = this.Mesh(iMesh);
                mf   = mesh.Assembly.ModeledFraction;
                I    = mesh.PeriodicBoundaryNodes(1, :);
                J    = mesh.PeriodicBoundaryNodes(2, :);
                N    = numel(mesh.X);
                K    = setdiff(1:N, [I, J]);
                
                if isstruct(mats)
                    field   = fields(mats);
                    nFields = numel(field);
                    
                    for iField = 1:nFields
                        if iscell(mats(iMesh).(field{iField}))
                            nCells = numel(mats(iMesh).(field{iField}));
                            for iCell = 1:nCells
                                mats(iMesh).(field{iField}){iCell} = this.pbcSubroutine(mats(iMesh).(field{iField}){iCell},action,I,J,K,sgn,mf);
                            end
                        else
                            mats(iMesh).(field{iField}) = this.pbcSubroutine(mats(iMesh).(field{iField}),action,I,J,K,sgn,mf);
                        end
                    end
                else
                    mats = this.pbcSubroutine(mats,action,I,J,K,sgn,mf);
                end
            end
        end
        
        function this = applyTangentialBoundaryConditions(this)
          	%% apply periodic boundary conditions to curl matrices
            %  remember in the three-dimensional case we will need to
            %  perform a rotation of the x and y components of the element
            %  to node curl matrices
            
            mesh  = this.Mesh;
            index = this.Index;
            nMesh = numel(mesh);
            for i = 1:nMesh
                J = mesh(i).PeriodicBoundaryNodes(2,:);
                
            	%% Indices
                %TODO - Create general function to remove indices / nodes');
                    
                index.Local(i).A(J)  = [];
                index.Global(i).A(J) = [];
                d = cumsum(diff(index.Local(i).A) - 1);
                d = [0,d] + min(index.Local(i).A) - 1;
                
                index.Local(i).A  = index.Local(i).A  - d;
                index.Global(i).A = index.Global(i).A - d;
                
                if i ~= 1
                    index.Global(i).A = index.Global(i).A - (min(index.Global(i).A) - max(index.Global(i-1).A) - 1);
                end
                
                index.Local(i).X(J)  = [];
                index.Global(i).X(J) = [];
                d = cumsum(diff(index.Local(i).X) - 1);
                d = [0,d] + min(index.Local(i).X) - 1;
                
                index.Local(i).X  = index.Local(i).X  - d;
                index.Global(i).X = index.Global(i).X - d;
                
                if i ~= 1
                    index.Global(i).X = index.Global(i).X - (min(index.Global(i).X) - max(index.Global(i-1).X) - 1);
                end
                
                index.Local(i).Unknowns = index.Local(i).Unknowns  - numel(J);
                
                for j = i:nMesh
                    index.Global(j).Unknowns = index.Global(j).Unknowns - numel(J);
                end
                
             	index.Local(i).Regions  = index.Local(i).Regions  - numel(J);
                index.Global(i).Regions = index.Global(i).Regions - numel(J);
            end
            
            this.Index = index;
        end
        
        function this = applyRadialBoundaryConditions(this)
            for i = 1:numel(this.Mesh)
                M =   real(this.Boundary.Radial.S(i).Inner * this.Boundary.Radial.F(i).Inner * this.Boundary.Radial.D(i).Inner) ...
                    + real(this.Boundary.Radial.S(i).Outer * this.Boundary.Radial.F(i).Outer * this.Boundary.Radial.D(i).Outer);
                M = (M + M') / 2;
                this.Stiffness.Reluctivity(i).Kff = this.Stiffness.Reluctivity(i).Kff + M;
            end
        end
        
        %% Master Loop Function
      	function structure = buildMatrices(this, structure, methodName)
            for iMesh = numel(this.Mesh):-1:1;
                [structure, newFields, pbcType] = this.(methodName)(structure, iMesh);
                for iField = 1:numel(newFields)
                    structure.(newFields{iField}) = applyPeriodicBoundaryConditions(this, structure.(newFields{iField}), iMesh, pbcType{iField});
                end
            end
        end
    end
    
    methods %AuxillaryFunctions
        function [y, y_t] = doPostProcessing(this, x, x_t)
            %% Recover Full Solution
            nTimes      = numel(x);
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            y           = cell(nAssemblies,nTimes);
            y_t         = cell(nAssemblies,nTimes);
            index       = this.Index;
            for i = 1:nAssemblies
                I = index.Global(i).X;
                for j = 1:nTimes
                    y{i,j}   = ppMatrices(i).Reduced2Full * x{j}(I);
                    y_t{i,j} = ppMatrices(i).Reduced2Full * x_t{j}(I);
                end
            end
        end
        
        function v = sobolev(this, x)
            index    = this.Index;
            curlN2E  = this.Jacobian.FluxDensity;
            post     = this.PostProcessing;
            
            v = 0;
            for i = 1:2
                I = index.Global(i).X;
                z = x(I);
                
                Bx = curlN2E(i).dBxdXz * z;
                By = curlN2E(i).dBydXz * z;
                v = v + Bx.' * post(i).SobolevB * Bx;
                v = v + By.' * post(i).SobolevB * By;
                
                z = post(i).Reduced2Full * z;
                v = v + z.' * post(i).SobolevA * z;
            end
            v = sqrt(v);
        end
        
        function [t, h] = getTimePoints(this, Nt)
            %TODO - Present implementation assumes synchronous operation
            [t,h] = this.Assemblies.getTimePoints(Nt);
        end
        
        function I = getGlobalIndex(this, var)
            index = this.Index.Local;
            
            I = [index(1).(var)];
            n = index(1).Unknowns;
            for i = 2:length(index);
                I = [I,  index(i).(var)+n];
                n = n + index(i).Unknowns;
            end
        end
        
        function I = getLocalIndex(this, var)
            index = this.Index.Local;
            m = length(index);
            
            I = cell(1, m);
            I{1} = index(1).(var);
            n = index(1).Unknowns;
            for i = 2:m
                I{i} = index(i).(var) + n;
                n = n + index(i).Unknowns;
            end
        end
        
        function harmonics = getRadialBoundaryHarmonics(this, iMesh)
           	%% Determine the number of boundaries and edges
            edges       = [this.Mesh.RadialBoundaryEdges];
            nBoundaries = numel(edges);
            nEdges      = cellfun('prodofsize',edges) / 2;
            
            %% Determine the range and spacing of spatial harmonics to include in the Fourier expansion
            nMin = this.Model.SpatialSymmetries;
            if this.Model.HasHalfWaveSymmetry
                hasHalfWaveSymmetry = true;
                dn = 2 * nMin;
            else
                hasHalfWaveSymmetry = false;
                dn = nMin;
            end
            nMax = 2 * nEdges * nMin;
            
            %% Calculate harmonics
            harmonics = cell(1,nBoundaries);
            I         = nMin:dn:nMax(1);
            if hasHalfWaveSymmetry
                harmonics{1} = [-fliplr(I),I];
            else
                harmonics{1} = [-fliplr(I),0,I];
            end
            
            for i = 2:2:(nBoundaries - 1)
                I              = nMin:dn:max(nMax(i:i+1));
                if hasHalfWaveSymmetry
                    harmonics{i}   = [-fliplr(I),I];
                else
                    harmonics{i}   = [-fliplr(I),0,I];
                end
                
                harmonics{i+1} = harmonics{i};
            end
            
            I = nMin:dn:nMax(end);
            if hasHalfWaveSymmetry
                harmonics{end} = [-fliplr(I),I];
            else
                harmonics{end} = [-fliplr(I),0,I];
            end
            
            if nargin == 2
                harmonics = harmonics((2*iMesh-1):(2*iMesh));
            end
        end
        
     	function gHandleOut = plot(this)
            regions  = this.Regions_;
            nRegions = numel(regions);
            elements = this.Elements;
            triX     = this.X(elements);
            triY     = this.Y(elements);
            elRegion = this.ElementRegions;
            fillArgs = cell(3*nRegions,1);

            for iRegion = 1:nRegions
                elInRegion        	 = (elRegion == iRegion);
                iFillArg             = (iRegion-1)*3;
                fillArgs{iFillArg+1} = triX(:,elInRegion);
                fillArgs{iFillArg+2} = triY(:,elInRegion);
                fillArgs{iFillArg+3} = regions(iRegion).Material.Color;
            end
            
            if nargout == 1
                gHandleOut = fill(fillArgs{:});
            else
                fill(fillArgs{:});
            end
            
            axis equal;
        end
    end
    
    methods (Static) %Apply Boundary Conditions
        function mats = pbcSubroutine(mats,action,I,J,K,sgn,mf)
            switch lower(action)
                case {'row', 'rows'}
                    mats(I,:) = mats(I,:) + sgn * mats(J,:);
                    mats(J,:) = [];
                case {'column', 'columns'}
                    mats(:,I) = mats(:,I) + sgn * mats(:,J);
                    mats(:,J) = [];
                case 'both'
                    mats(I,I) = mats(I,I) + mats(J,J);
                    mats(I,K) = mats(I,K) + sgn * mats(J,K);
                    mats(K,I) = mats(K,I) + sgn * mats(K,J);
                    mats(J,:) = [];
                    mats(:,J) = [];
                case 'reducerow'
                    mats(J,:) = [];
                case 'restorerow'
                    mats(J,:) = sgn * mats(I,:);
                    mats(:,J) = [];
                case 'boundaryoperation'
                    mats      = mats / mf;
                    mats(:,I) = mats(:,I) + sgn * mats(:,J);
                    mats(:,J) = [];
                case 'inverseboundaryoperation'
                    mats      = mats * mf;
                    mats(I,:) = (mats(I,:) + sgn * mats(J,:))/2;
                    mats(J,:) = [];
                otherwise
                    error('MotorProto:MatrixFactory', 'Unknown argument "%s" for method applyPeriodicBoundaryConditions', action);
            end
        end
    end
    
    methods (Abstract) %Matrix and Vector Functions
        linearMatrix       = K(this,t,h)
        
        conductivityMatrix = C(this,t,h)
        
        exogenousFunction  = f(this,t,h)
        
        [nonlinearJacobian, nonlinearFunction] = G(this,t,x)
    end

    methods %Post Processing - %TODO - Seperate post processing into another class
       	%% Field Variables
        function [x, x_t, labels, text, nTimes] = continuumVariablePreProcessing(~, solver, dataType, dataPoints)
         	times = solver.Times;
            
            switch lower(dataType)
                case 'time'
                    x      = solver.X(:,dataPoints);
                    x_t    = solver.X_t(:,dataPoints);
                    labels = times(dataPoints);
                    nTimes = numel(dataPoints);
                    text   = 't = %0.4g';
                case 'harmonic'
                	x      = solver.X;
                    x_t    = solver.X_t;
                    labels = dataPoints;
                    nTimes = numel(times);
                    text   = 'h = %d';
                otherwise
                    x      = solver.X;
                    x_t    = solver.X_t;
                    labels = '';
                    nTimes = numel(times);
                    text   = '';
            end
        end
        
        function [a, labels, text] = A(this, solver, dataType, dataPoints)
            [x, ~, labels, text, nTimes] = continuumVariablePreProcessing(this, solver, dataType, dataPoints);
            
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            a           = cell(nAssemblies, nTimes);
            nRows       = zeros(nAssemblies, 1);
            for i = 1:nAssemblies
                I = ppMatrices(i).X2A;
                for j = 1:nTimes
                    a{i,j} = I * x{i,j};
                end
                nRows(i) = length(a{i,1});
            end
            
            if strcmpi(dataType,'harmonic')
                a = cell2mat(a(:,1:(end-1)));
                a = fft(a,[],2) / (nTimes - 1);
                a = mat2cell(a,nRows,ones(1,nTimes-1));
                a = a(:,mod(dataPoints,nTimes - 1) + 1);
            end
        end
        
        function [e, labels, text] = E(this, solver, dataType, dataPoints)
            [x, x_t, labels, text, nTimes] = continuumVariablePreProcessing(this,solver,dataType,dataPoints);
            
            if strcmpi(dataType,'time')
                t = labels;
            else
                t = solver.Times;
            end
            
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            e           = cell(nAssemblies,nTimes);
            assembly    = this.Assemblies;
            
            nRows       = zeros(nAssemblies,1);
          	nVertPerEl  = numel(ppMatrices(1).X_t2E);
            
            for i = 1:nAssemblies
                for j = 1:nTimes
                    e{i,j} = ppMatrices(i).X2E * x{i,j};

                    circuits = assembly(i).Circuits;
                    if numel(circuits) > 0
                        e{i,j} = e{i,j} + ppMatrices(i).F2E * circuits.f(t(j));
                    end

                    e{i,j}     = repmat(e{i,j},1,nVertPerEl);

                    for k = 1:nVertPerEl
                        e{i,j}(:,k) = e{i,j}(:,k) + ppMatrices(i).X_t2E{k} * x_t{i,j};
                    end
                end
                nRows(i) = length(e{i,1});
            end
            
            if strcmpi(dataType,'harmonic')
                e = cellfun(@(x)(reshape(x,[],1)),e,'UniformOutput',false);
                e = cell2mat(e(:,1:(end-1)));
                e = fft(e,[],2) / (nTimes - 1);
                e = mat2cell(e,nRows*nVertPerEl,ones(1,nTimes-1));
                e = e(:,mod(dataPoints,nTimes - 1) + 1);
                e = cellfun(@(x)(reshape(x,[],nVertPerEl)),e,'UniformOutput',false);
            end
        end
        
        function [j, labels, text] = J(this, solver, dataType, dataPoints)
            [x, x_t, labels, text, nTimes] = continuumVariablePreProcessing(this,solver,dataType,dataPoints);
            
            if strcmpi(dataType,'time')
                t = labels;
            else
                t = solver.Times;
            end
            
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            j           = cell(nAssemblies,nTimes);
            assembly    = this.Assemblies;
            nRows       = zeros(nAssemblies,1);
            nVertPerEl  = numel(ppMatrices(1).X_t2J);
            
            for i = 1:nAssemblies
                for k = 1:nTimes
                    j{i,k} = ppMatrices(i).X2J * x{i,k};

                    circuits = assembly(i).Circuits;
                    if numel(circuits) > 0
                        j{i,k} = j{i,k} + ppMatrices(i).F2J * circuits.f(t(k));
                    end

                    j{i,k} = repmat(j{i,k},1,nVertPerEl);

                    for l = 1:nVertPerEl
                        j{i,k}(:,l) = j{i,k}(:,l) + ppMatrices(i).X_t2J{l} * x_t{i,k};
                    end
                end
                nRows(i) = length(j{i,1});
            end

         	if strcmpi(dataType,'harmonic')
                j = cellfun(@(x)(reshape(x,[],1)),j,'UniformOutput',false);
                j = cell2mat(j(:,1:(end-1)));
                j = fft(j,[],2) / (nTimes - 1);
                j = mat2cell(j,nRows*nVertPerEl,ones(1,nTimes-1));
                j = j(:,mod(dataPoints,nTimes - 1) + 1);
                j = cellfun(@(x)(reshape(x,[],nVertPerEl)),j,'UniformOutput',false);
            end
        end
        
        function [b, labels, text] = B(this, solver, dataType, dataPoints)
            [bx, by, labels, text] = BVector(this, solver, dataType, dataPoints);
            
            b = cellfun(@(x,y)(hypot(x,y)),bx,by,'UniformOutput',false);
        end
        
        function [bx, by, labels, text] = BVector(this, solver, dataType, dataPoints)
            [x, ~, labels, text, nTimes] = continuumVariablePreProcessing(this, solver, dataType, dataPoints);
            
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            bx          = cell(nAssemblies,nTimes);
            by          = cell(nAssemblies,nTimes);

            del         = this.Del;
            nRows       = zeros(nAssemblies,1);
            for i = 1:nAssemblies
                for j = 1:nTimes
                    bx{i,j} = del.Curl(i).Zn2Xe * x{i,j};
                    by{i,j} = del.Curl(i).Zn2Ye * x{i,j};
                end
                nRows(i) = length(bx{i,1});
            end
            
         	if strcmpi(dataType,'harmonic')
                bx = cell2mat(bx(:, 1:(end-1)));
                %bx = fft(bx, [], 2) / (nTimes - 1);
                bx = solver.fft(bx);
                bx = mat2cell(bx, nRows, ones(1,nTimes-1));
                bx = bx(:,mod(dataPoints, nTimes - 1) + 1);
                
                by = cell2mat(by(:, 1:(end-1)));
                %by = fft(by, [], 2) / (nTimes - 1);
                by = solver.fft(by);
                by = mat2cell(by, nRows, ones(1,nTimes-1));
                by = by(:, mod(dataPoints, nTimes - 1) + 1);
            end
        end
        
       	function [m, labels, text] = M(this, solver, dataType, dataPoints)
            [mx, my, labels, text] = MVector(this, solver, dataType, dataPoints);
            
            m = cellfun(@(x,y)(hypot(x,y)),mx,my,'UniformOutput',false);
        end
        
        function [mx, my, labels, text] = MVector(this, solver, dataType, dataPoints)
            [x, ~, labels, text, nTimes] ...
                = continuumVariablePreProcessing(this,solver,dataType,dataPoints);
            
            ppMatrices  = this.PostProcessing;
            nAssemblies = numel(ppMatrices);
            
            mx          = cell(nAssemblies,nTimes);
            my          = cell(nAssemblies,nTimes);
            
            del         = this.Del;
            assembly    = this.Assemblies;
            mesh        = this.Mesh;
            nRows       = zeros(nAssemblies,1);
            
            for i = 1:nAssemblies
                for j = 1:nTimes
                    bx = del.Curl(i).Zn2Xe * x{i,j};
                    by = del.Curl(i).Zn2Ye * x{i,j};
                    s  = size(bx);

                    mx{i,j} = zeros(s);
                    my{i,j} = zeros(s);
                    materials = [assembly(i).Regions.Material];
                    for k = 1:numel(materials)
                        K = (mesh(i).ElementRegions == k);
                        [mx{i,j}(K),my{i,j}(K)] = materials(k).vectorM(bx(K),by(K));
                    end
                    [~,rMx,rMy] = materials.vectorMr;
                    mx{i,j}     = mx{i,j} + rMx(mesh(i).ElementRegions);
                    my{i,j}     = my{i,j} + rMy(mesh(i).ElementRegions);
                end
                nRows(i) = length(mx{i,1});
            end
            
         	if strcmpi(dataType,'harmonic')
                mx = cell2mat(mx(:,1:(end-1)));
                %mx = fft(mx,[],2) / (nTimes - 1);
                mx = solver.fft(mx);
                mx = mat2cell(mx,nRows,ones(1,nTimes-1));
                mx = mx(:,mod(dataPoints,nTimes - 1) + 1);
                
                my = cell2mat(my(:,1:(end-1)));
                %my = fft(my,[],2) / (nTimes - 1);
                my = solver.fft(my);
                my = mat2cell(my,nRows,ones(1,nTimes-1));
                my = my(:,mod(dataPoints,nTimes - 1) + 1);
            end
        end
        
        function [hx, hy, labels, text] = HVector(this, solver, dataType, dataPoints)
            [bx, by, labels, text] = BVector(this, solver, dataType, dataPoints);
            [mx, my]               = MVector(this, solver, dataType, dataPoints);
            
            hx = cellfun(@(b,m)(b/mu_o - m), bx, mx, 'UniformOutput',false);
            hy = cellfun(@(b,m)(b/mu_o - m), by, my, 'UniformOutput',false);
        end
        
     	function [h, labels, text] = H(this, solver, dataType, dataPoints)
            [hx, hy, labels, text] = HVector(this, solver, dataType, dataPoints);
            
            h = cellfun(@(x, y)(hypot(x, y)), hx, hy, 'UniformOutput', false);
        end
        
        %% Density Variables
        function [l, labels, text] = AverageLossDensity(this, solver, varargin)
            [lcond, lcondtot] = AverageConductionLossDensity(this, solver);
            [lcore, lcoretot] = AverageCoreLossDensity(this, solver);
            
            l = lcond;
            for i = 1:numel(l)
                l{i} = bsxfun(@plus, l{i}, lcore{i});
                l{i}(l{i} == 0) = NaN;
            end
            
            text   = '(P_{loss} = %0.3g W)';
            labels = lcondtot + lcoretot;
        end
        
        function [l, labels, text] = AverageConductionLossDensity(this, solver, varargin)
            l      = ConductionLossDensity(this, solver, 'Harmonic', 0);
            text   = '(P_{cond} = %0.3g W)';
            labels = AverageConductionLosses(this, solver, 'time');
            labels = labels{1};
        end
        
        function [l, labels, text] = AverageCoreLossDensity(this, solver, varargin)
            pHarmonics = CoreLossDensity(this, solver);
            text       = '(P_{core} = %0.3g W)';
            
            mesh   = this.Mesh;
            nMesh  = numel(mesh);
            l      = cell(nMesh,1);
            labels = zeros(1,nMesh);
            for i = 1:nMesh
                l{i}      = sum([pHarmonics{i,:}],2);
                labels(i) = mesh(i).ElementAreas * l{i} * this.Model.Assemblies(i).Length / this.Model.Assemblies(i).ModeledFraction;
            end
        end
                
      	function [l, labels, text] = ConductionLossDensity(this, solver, dataType, dataPoints)
            %TODO - Need to take into account shape functions when multiplying E and J. Quadratic terms are introduced
                
            [~, ~, text,labels] = continuumVariablePreProcessing(this, solver, dataType, dataPoints);
            switch lower(dataType)
                case 'time'
                    e = E(this, solver, 'time', dataPoints);
                    j = J(this, solver, 'time', dataPoints);
                case 'harmonic'
                    [~, text, labels] = E(this, solver, 'harmonic', dataPoints);
                    times  = solver.Times;
                    nTimes = numel(times);
                    e = E(this, solver, 'time', 1:nTimes);
                    j = J(this, solver, 'time', 1:nTimes);
            end
            l     = cellfun(@(x,y)(x.*y),e,j,'UniformOutput',false);
            nRows = cellfun('length',l(:,1));
            [~,nVertPerEl] = size(l{1,1});
            
            if strcmpi(dataType,'harmonic')
                l = cellfun(@(x)(reshape(x,[],1)),l,'UniformOutput',false);
                l = cell2mat(l(:,1:(end-1)));
                %l = fft(l,[],2) / (nTimes - 1);
                l = solver.fft(l);
                l = mat2cell(l,nRows*nVertPerEl,ones(1,nTimes-1));
                l = l(:,mod(dataPoints,nTimes - 1) + 1);
                l = cellfun(@(x)(reshape(x,[],nVertPerEl)),l,'UniformOutput',false);
            end
        end
        
        function [l, labels, text] = CoreLossDensity(this, solver, varargin)
            times      = solver.Times;
            nHarmonics = numel(times - 1) / 2 - 1;
            h          = 0:(nHarmonics - 1);
            
            [b, text, labels] = this.B(solver, 'Harmonic', h);
            f_e = this.Model.TemporalFrequency / this.Model.TemporalSubharmonics;
            
            assert( all(f_e - mean(f_e) < sqrt(eps) * mean(f_e)) || all(f_e == 0), 'MotorProto:StaticMatrixFactory', 'All electrical frequencies should be identical');
            
            f_e = mean(f_e);
            f   = f_e * h;
            
            mesh   = this.Mesh;
            nMesh  = numel(mesh);
            l      = cellfun(@(x)(x*0),b,'UniformOutput',false);
            for i = 1:nMesh
                material   = mesh(i).Materials;
                nMaterials = numel(material);
                for j = 1:nMaterials
                  	J = (mesh(i).ElementRegions == j);
                    if ~material(j).Linear
                        s = material(j).CoreLossCoefficients;
                        for k = 1:nHarmonics
                            l{i,k}(J) = l{i,k}(J)+s(1)*f(k)^s(2)*(2*b{i,k}(J)).^s(3);
                        end
                    else
                        for k = 1:nHarmonics
                            l{i,k}(J) = 0;
                        end
                    end
                end
            end            
        end
        
        %% Bulk Power Variables
        function [p, figLabels, figTitles] = AverageLosses(this, solver, dataType)
            if nargin < 3
                dataType = [];
            end
            
            p_cond = AverageConductionLosses(this, solver, dataType);
            p_core = AverageCoreLosses(this, solver, dataType);
            p      = sum(p_cond{1}) + sum(p_core{1});
            
            p         = {p};
            figLabels = {''};
            figTitles = {''};
        end

        function [l, figLabels, figTitles] = AverageCoreLosses(this, solver, dataType)            
            switch lower(dataType)
                case {'default','time'}
                    pHarmonics = CoreLossDensity(this, solver);
                    mesh   = this.Mesh;
                    nMesh  = numel(mesh);
                    l      = zeros(1,nMesh);
                    for i = 1:nMesh
                        l(i) = mesh(i).ElementAreas * sum([pHarmonics{i,:}],2) * this.Model.Assemblies(i).Length / this.Model.Assemblies(i).ModeledFraction;
                    end
            
                    l         = {l};
                    figLabels = {''};
                    figTitles = {''};
                case 'harmonic'
                    error('No Implementation');
            end
        end
        
     	function [l, figLabels, figTitles] = InstantaneousConductionLosses(this, solver, dataType)
            x           = solver.X;
            x_t         = solver.X_t;
            t           = solver.Times;
            Nt          = numel(t);
            mesh        = this.Mesh;
            assembly    = this.Assemblies;
            nAssemblies = numel(assembly);
            l           = zeros(nAssemblies,numel(t));
            ppMatrix    = this.PostProcessing;
            
            for p = 1:nAssemblies;
                circuits   = [assembly(p).Circuits];
                nSources = numel(circuits);
                ela      = mesh(p).ElementAreas.';
                for k = 1:Nt;
                    e0 = ppMatrix(p).X2E * x{p,k};
                    j0 = ppMatrix(p).X2J * x{p,k};
                    
                    if nSources > 0
                        e0 = e0 + ppMatrix(p).F2E * circuits.f(t(k));
                        j0 = j0 + ppMatrix(p).F2J * circuits.f(t(k));
                    end
                    
                    ne = numel(ppMatrix(p).X_t2E);
                    nj = numel(ppMatrix(p).X_t2J);

                    e1 = zeros(size(e0));
                    for i = 1:ne
                        e1(:,i) = ppMatrix(p).X_t2E{i} * x_t{p,k};
                    end

                    j1 = zeros(size(j0));
                    for i = 1:nj
                        j1(:,i) = ppMatrix(p).X_t2J{i} * x_t{p,k};
                    end

                    %% 0 x 0
                    l(p,k) = j0'*(ela.*e0);

                    %% 1 x 0, 0 x 1
                    for i = 1:ne
                        c = 2 * factorial(0) * factorial(1) / factorial(2 + 0 + 1);
                        l(p,k) = l(p,k) + c*j0'*(ela.*e1(:,i));
                    end

                    for i = 1:nj
                        c = 2 * factorial(0) * factorial(1) / factorial(2 + 0 + 1);
                        l(p,k) = l(p,k) + c*j1(:,i)'*(ela.*e0);
                    end

                    %% 1 x 1
                    for i = 1:ne
                        for j = 1:ne
                            if i == j
                                ik = 2;
                                jk = 0;
                            else
                                ik = 1;
                                jk = 1;
                            end
                            c = 2*factorial(ik)*factorial(jk)/factorial(2+ik+jk);
                            l(p,k) = l(p,k) + c*j1(:,j)'*(ela.*e1(:,i));
                        end
                    end
                end
                
                l(p,:) = l(p,:) * this.Model.Assemblies(p).Length / this.Model.Assemblies(p).ModeledFraction;
            end
            
            if strcmpi(dataType,'harmonic')
                %l = fft(l(:,1:(Nt-1)),[],2) / (Nt - 1);
                l = solver.fft(l(:,1:(Nt-1)));
            end
            
            l         = num2cell(l,2);
            figLabels = {''};
            figTitles = {''};
        end
        
        function [l, figLabels, figTitles] = AverageConductionLosses(this, solver, dataType)
            switch lower(dataType)
                case {'default','time'}
                    [l, figLabels, figTitles] = InstantaneousConductionLosses(this, solver, 'Time');
                    Nt = numel(solver.Times) - 1;
                    for i = 1:numel(l)
                        l{i} = solver.fft(l{i}(1:Nt));
                        l{i} = real(l{i}(1));
                    end
                    l = cell2mat(l).';
                    l = {l};
                case 'harmonic'
                   	x        = solver.X;
                    x_t      = solver.X_t;
                    t        = solver.Times;
                    Nt       = numel(t);
                    nRows    = cellfun('length', x);
                    nRows    = nRows(:,1);
                    
                    x        = cell2mat(x);
                    %x        = fft(full(x(:,1:(Nt-1))),[],2) / (Nt - 1);
                    x        = solver.fft(full(x(:,1:(Nt-1))));
                    x        = mat2cell(x,nRows,ones(1,Nt-1));
                    
                    x_t      = cell2mat(x_t);
                    %x_t      = fft(full(x_t(:,1:(Nt-1))),[],2) / (Nt - 1);
                    x_t      = solver.fft(full(x_t(:,1:(Nt-1))));
                    x_t      = mat2cell(x_t,nRows,ones(1,Nt-1));
                    
                    mesh     = this.Mesh;
                    assembly = this.Assemblies;
                    l        = zeros(1,numel(t));
                    ppMatrix = this.PostProcessing;

                    for p = 1:numel(ppMatrix);
                        circuit   = [assembly(p).Circuits];
                        f        = circuit.f(t);
                        f        = fft(f(:,1:(Nt-1)),[],2) / (Nt - 1);
                        nSources = numel(circuit);
                        ela      = mesh(p).ElementAreas.';
                        for k = 1:(Nt-1);
                            e0 = ppMatrix(p).X2E * x{p,k};
                            j0 = ppMatrix(p).X2J * x{p,k};

                            if nSources > 0
                                e0 = e0 + ppMatrix(p).F2E * f(:,k);
                                j0 = j0 + ppMatrix(p).F2J * f(:,k);
                            end

                            ne = numel(ppMatrix(p).X_t2E);
                            nj = numel(ppMatrix(p).X_t2J);

                            e1 = zeros(size(e0));
                            for i = 1:ne
                                e1(:,i) = ppMatrix(p).X_t2E{i} * x_t{p,k};
                            end

                            j1 = zeros(size(j0));
                            for i = 1:nj
                                j1(:,i) = ppMatrix(p).X_t2J{i} * x_t{p,k};
                            end

                            %% 0 x 0
                            l(k) = l(k) + j0'*(ela.*e0);

                            %% 1 x 0, 0 x 1
                            for i = 1:ne
                                c = 2 * factorial(0) * factorial(1) / factorial(2 + 0 + 1);
                                l(k) = l(k) + c*j0'*(ela.*e1(:,i));
                            end

                            for i = 1:nj
                                c = 2 * factorial(0) * factorial(1) / factorial(2 + 0 + 1);
                                l(k) = l(k) + c*j1(:,i)'*(ela.*e0);
                            end

                            %% 1 x 1
                            for i = 1:ne
                                for j = 1:ne
                                    if i == j
                                        ik = 2;
                                        jk = 0;
                                    else
                                        ik = 1;
                                        jk = 1;
                                    end
                                    c = 2*factorial(ik)*factorial(jk)/factorial(2+ik+jk);
                                    l(k) = l(k) + c*j1(:,j)'*(ela.*e1(:,i));
                                end
                            end
                        	l(k) = l(k) * this.Model.Assemblies(1).Length / this.Model.SpaceModelFraction;
                        end
                    end
                 	l         = {l};
                    figLabels = {''};
                    figTitles = {''};
            end    
        end
        
        function [tau, figLabels, figTitles] = Torque(this, solver, dataType)
            x   = solver.X;
            t   = solver.Times;
            ppm = this.PostProcessing;
            
            R = this.Boundary.Radial.R;
            D = this.Boundary.Radial.D;
            F = this.Boundary.Radial.F;
            G = this.Boundary.Radial.G;
            
            assemblies  = this.Assemblies;
            nAssemblies = numel(assemblies);
            tau         = cell(1,nAssemblies);
                
            for j = 1:nAssemblies
                nh = this.getRadialBoundaryHarmonics(j);
                ri = assemblies(j).InnerRadius;
                ro = assemblies(j).OuterRadius;
                l  = assemblies(j).Length;
                Nt = length(t);
                tau{j} = zeros(2,Nt);
                
                for i = 1:Nt
                    ht = (F(j).Inner * D(j).Inner * ppm(j).Full2Reduced * x{j,i});
                    if j > 1
                        ht = exp(1i * (R(j).Inner - R(j-1).Outer) * t(i)) .* (G(j-1).Outer * D(j-1).Outer * ppm(j-1).Full2Reduced * x{j-1,i});
                    end
                    br = 1i * nh{1}.' .* (D(j).Inner * ppm(j).Full2Reduced *  x{j,i}) / ri;
                    tau{j}(1,i) = 2*pi*l*ri^2*real(sum(conj(ht).*br));
                    
                    ht = (F(j).Outer * D(j).Outer * ppm(j).Full2Reduced * x{j,i});
                    if j < nAssemblies
                        ht = exp(1i * (R(j).Outer - R(j+1).Inner) * t(i)) .* (G(j+1).Inner * D(j+1).Inner * ppm(j+1).Full2Reduced * x{j+1,i});
                    end
                    br = 1i * nh{2}.' .* (D(j).Outer * ppm(j).Full2Reduced * x{j,i}) / ro;
                    tau{j}(2,i) = 2*pi*l*ro^2*real(sum(conj(ht).*br));
                end
                tau{j} = tau{j}(2,:)-tau{j}(1,:);
            end
            
            if strcmpi(dataType,'harmonic')
                for j = 1:nAssemblies
                    %tau{j} = fft(tau{j}(1:(Nt-1)),[],2) / (Nt - 1);
                    tau{j} = solver.fft(tau{j}(1:(Nt-1)));
                end
            end
            
            figLabels = cell(1,nAssemblies);
            figTitles = cell(1,nAssemblies);
            for i = 1:nAssemblies
                figLabels{i} = '';
                figTitles{i} = assemblies(i).Name;
            end
        end
        
        function [tau, figLabels, figTitles] = AverageTorque(this, solver, dataType)
        	% #TODO - Generalize for multiple annuli
            switch lower(dataType)
                case {'default','time'}
                    [tau, figLabels, figTitles] = Torque(this, solver, 'Time');
                    tau                         = {cellfun(@(x)(mean(x(1:(end-1)))), tau)};
                case 'harmonic'
                    error('No Implementation');
            end
        end
        
        function [pow, figLabels, figTitles] = AverageOutputPower(this, solver, dataType)
        	% #TODO - Generalize for multiple annuli
            switch lower(dataType)
                case {'default','time'}
                    [tau, figLabels, figTitles] = AverageTorque(this, solver, 'Time');
                    pow                         = {tau{1} * this.Assemblies(1).AngularVelocity};
                case 'harmonic'
            end
        end
        
        function [eff, figLabels, figTitles] = Efficiency(this, solver, dataType)
        	% #TODO - Generalize for multiple annuli
            if nargin < 3
                dataType = [];
            end
            
            p_loss = AverageLosses(this, solver, dataType);
            p_out  = AverageOutputPower(this, solver, dataType);
            
            eff       = {p_out{1} / (p_out{1} + p_loss{1})};
            figLabels = {''};
            figTitles = {''};
        end
        
        %% Other Bulk Variables
        function [v, figLabels, figTitles] = Voltage(this, solver, dataType)
            x           = solver.X;
            x_t         = solver.X_t;
            t           = solver.Times;
            Nt          = numel(t);
            assembly    = this.Assemblies;
            nAssemblies = numel(assembly);
            v           = cell(nAssemblies, 1);
            figLabels   = cell(nAssemblies, 1);
            figTitles   = cell(nAssemblies, 1);
            remove      = false(nAssemblies, 1);
            ppMatrices  = this.PostProcessing;
            
            for i = 1:nAssemblies                
                circuit    = assembly(i).Circuits;
                hasCircuit = (numel(circuit) > 0);
                if hasCircuit
                    figTitles{i} = circuit.getTitle('voltage');
                    figLabels{i} = circuit.getLabels('voltage');
                    
                    v{i} = ppMatrices(i).X2V * x{i,1};
                    v{i} = v{i} + ppMatrices(i).X_t2V * x_t{i,1};
                	v{i} = v{i} + ppMatrices(i).F2V * circuit.f(t(1));
                    v{i} = repmat(v{i},1,Nt);
                    for j = 2:Nt
                        v{i}(:,j) = ppMatrices(i).X2V * x{i,j};
                        v{i}(:,j) = v{i}(:,j) + ppMatrices(i).X_t2V * x_t{i,j};
                      	v{i}(:,j) = v{i}(:,j) + ppMatrices(i).F2V * circuit.f(t(j));
                    end
                
                    if strcmpi(dataType,'harmonic')
                        %v{i} = fft(v{i}(:,1:(end-1)), [] ,2) / (Nt - 1);
                        v{i} = solver.fft(v{i}(:,1:(end-1)));
                    end
                    v{i} = num2cell(v{i},2);
                else
                    remove(i) = true;
                end
            end
            v(remove)         = [];
            figLabels(remove) = [];
            figTitles(remove) = [];
        end
        
        function [lambda, figLabels, figTitles] = FluxLinkage(this, solver, dataType)
            x           = solver.X;
            t           = solver.Times;
            Nt          = numel(t);
            assembly    = this.Assemblies;
            nAssemblies = numel(assembly);
            lambda      = cell(nAssemblies, 1);
            figLabels   = cell(nAssemblies, 1);
            figTitles   = cell(nAssemblies, 1);
            remove      = false(nAssemblies, 1);
            ppMatrices  = this.PostProcessing;
            
            for i = 1:nAssemblies                
                circuit   = assembly(i).Circuits;
                hasCircuit = (numel(circuit) > 0);
                if hasCircuit
                    figTitles{i} = circuit.getTitle('flux linkage');
                    figLabels{i} = circuit.getLabels('flux linkage');
                    
                    lambda{i} = ppMatrices(i).X2Lambda * x{i,1};
                    lambda{i} = repmat(lambda{i},1,Nt);
                    for j = 2:Nt
                        lambda{i}(:,j) = ppMatrices(i).X2Lambda * x{i,j};
                    end
                
                    if strcmpi(dataType,'harmonic')
                        %lambda{i} = fft(lambda{i}(:,1:(end-1)), [] ,2) / (Nt - 1);
                        lambda{i} = solver.fft(lambda{i}(:,1:(end-1)));
                    end
                    lambda{i} = num2cell(lambda{i},2);
                else
                    remove(i) = true;
                end
            end
            lambda(remove)    = [];
            figLabels(remove) = [];
            figTitles(remove) = [];
        end
        
        function [i, figLabels, figTitles] = Current(this, solver, dataType)
            x           = solver.X;
            x_t         = solver.X_t;
            t           = solver.Times;
            Nt          = numel(t);
            assembly    = this.Assemblies;
            nAssemblies = numel(assembly);
            i           = cell(nAssemblies, 1);
            figLabels   = cell(nAssemblies, 1);
            figTitles   = cell(nAssemblies, 1);
            remove      = false(nAssemblies, 1);
            ppMatrices  = this.PostProcessing;
            
            for j = 1:nAssemblies
                circuit   = assembly(j).Circuits;
                hasCircuit = (numel(circuit) > 0);

                if hasCircuit
                    figTitles{j} = circuit.getTitle('current');
                    figLabels{j} = circuit.getLabels('current');
                    
                    i{j} = ppMatrices(j).X2I * x{j,1};
                    i{j} = i{j} + ppMatrices(j).X_t2I * x_t{j,1};
                    i{j} = i{j} + ppMatrices(j).F2I * circuit.f(t(1));
                    i{j} = repmat(i{j},1,Nt);
                    for k = 2:Nt
                        i{j}(:,k) = ppMatrices(j).X2I * x{j,k};
                        i{j}(:,k) = i{j}(:,k) + ppMatrices(j).X_t2I * x_t{j,k};
                        i{j}(:,k) = i{j}(:,k) + ppMatrices(j).F2I * circuit.f(t(k));
                    end
                    
                    if strcmpi(dataType, 'harmonic')
                        %i{j} = fft(i{j}(:,1:(end-1)), [] ,2) / (Nt - 1);
                        i{j} = solver.fft(i{j}(:,1:(end-1)));
                    end
                    i{j} = num2cell(i{j},2);
                else
                    remove(j) = true;
                end
            end
            i(remove)         = [];
            figLabels(remove) = [];
            figTitles(remove) = [];
        end
    end
    
 	methods (Sealed) %Copy
        function copyOut = copy(this)
            nThis   = numel(this);
            copyOut = this;
            for i = 1:nThis
                copyOut(i).Model = copy(this.Model);
            end
        end
    end
end