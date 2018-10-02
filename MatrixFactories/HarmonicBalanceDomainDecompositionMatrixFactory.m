classdef HarmonicBalanceDomainDecompositionMatrixFactory < DynamicMatrixFactory
    methods
        %% Constructor
        function this = HarmonicBalanceDomainDecompositionMatrixFactory(varargin)
            if nargin > 0
                this.Model = varargin{1};
                this = build(this,'space');
            end
        end
        
        function this = partitionDomains(this)
            mesh            = this.Mesh;
            nMesh           = length(mesh);
            domainPartition = struct([]);
            for iMesh = 1:nMesh
                mesh = this.Mesh_(iMesh);
                P    = mesh.PeriodicBoundaryNodes(2,:);
                
                mats        = mesh.Materials;
                matIsLinear = [mats.Linear];
                el1         = matIsLinear(mesh.ElementRegions);    
                
%                 regs        = mesh.Regions;
%                 matIsStatic = ([regs.Dynamics] == DynamicsTypes.Static);
%                 el1         = ~matIsStatic(mesh.ElementRegions);
                
                domainPartition(iMesh).Domains            = 2;
                domainPartition(iMesh).Domain(1).Elements = el1;
                domainPartition(iMesh).Domain(2).Elements = ~el1;
                
                ind       = this.Index.Local(iMesh);
                nUnknowns = ind.Unknowns;
                
                I1 = unique(mesh.Elements(:, el1));
                I1 = setdiff(1:length(mesh.X),I1);
                I1 = setdiff(I1,ind.Boundary.Radius(1).Nodes);
                I1 = setdiff(I1,ind.Boundary.Radius(2).Nodes);
                
                J1            = true(1,nUnknowns);
                J1(ind.A(I1)) = false;
                J1(P)         = [];
                
                I2 = unique(mesh.Elements(:,~el1));
                I2 = union(I2,ind.Boundary.Radius(1).Nodes);
                I2 = union(I2,ind.Boundary.Radius(2).Nodes);

                J2            = false(1,nUnknowns);
                J2(ind.A(I2)) = true;
                J2(P)         = [];
                
                domainPartition(iMesh).Domain(1).Unknowns = J1;
                domainPartition(iMesh).Domain(2).Unknowns = J2;
            end
            
            this.DomainPartitions = domainPartition;
        end
        
        function this = applyRadialBoundaryConditions(this)
            for i = 1:numel(this.Mesh_)
                this.Stiffness.Reluctivity(i).El2El{1} ...
                    = this.Stiffness.Reluctivity(i).El2El{1} ...
                        + real(this.Boundary.Radial.S(i).Inner * this.Boundary.Radial.F(i).Inner * this.Boundary.Radial.D(i).Inner) ...
                        + real(this.Boundary.Radial.S(i).Outer * this.Boundary.Radial.F(i).Outer * this.Boundary.Radial.D(i).Outer);
            end
        end
        
        function [linearPart,nonlinearPart] = K(this, t, ~)
            assembly      = this.Assemblies_;
            nAssemblies   = numel(assembly);
%             linearPart    = cell(nAssemblies,nAssemblies);
            linearPart    = cell(1,nAssemblies);
            nTimes        = numel(t);
            nonlinearPart = cell(1,nTimes);
            
            %% Linear Variables
            for i = 1:nAssemblies
                linearPart{1,i} =   this.Stiffness.Reluctivity(i).El2El{1} ...
                                    + this.Stiffness.Reluctivity(i).Re2El    ...
                                    + this.Stiffness.Reluctivity(i).El2Re    ...
                                    + this.Stiffness.Reluctivity(i).Re2Re    ...
                                    + this.Stiffness.Reluctivity(i).Re2Cr    ...
                                    + this.Stiffness.Reluctivity(i).Cr2Re    ...
                                    + this.Stiffness.Reluctivity(i).Cr2Cr;

%                 %% locked rotor only
%                 if i > 1
%                     linearPart{i,i-1} = real(this.Boundary.Radial.S(i).Inner * this.Boundary.Radial.G(i-1).Outer * this.Boundary.Radial.D(i-1).Outer);
%                 end
%                 
%                 if i < nAssemblies
%                     linearPart{i,i+1} = real(this.Boundary.Radial.S(i).Outer * this.Boundary.Radial.G(i+1).Inner * this.Boundary.Radial.D(i+1).Inner);
%                 end
            end
%             linearPart = cell2mat(linearPart);
            linearPart = blkdiag(linearPart{:});

            %% Nonlinear Variables
            npMatPart = cell(nAssemblies,nAssemblies);
            for i = 1:nAssemblies
                npMatPart{i,i} = this.Stiffness.Reluctivity(i).El2El{2};
            end
            [nonlinearPart{:}] = deal(npMatPart);
            
            for j = 1:nTimes
                for i = 1:nAssemblies
                    if i > 1
                        R = this.Boundary.Radial.R(i).Inner - this.Boundary.Radial.R(i-1).Outer;
                        R = sparse(1:numel(R), 1:numel(R), exp(1i * R * t(j)));

                        nonlinearPart{j}{i,i-1} = real(this.Boundary.Radial.S(i).Inner * this.Boundary.Radial.G(i-1).Outer * R * this.Boundary.Radial.D(i-1).Outer);
                    end

                    if i < nAssemblies
                        R = this.Boundary.Radial.R(i).Outer - this.Boundary.Radial.R(i+1).Inner;
                        R = sparse(1:numel(R), 1:numel(R), exp(1i * R * t(j)));

                        nonlinearPart{j}{i,i+1} = real(this.Boundary.Radial.S(i).Outer * this.Boundary.Radial.G(i+1).Inner * R * this.Boundary.Radial.D(i+1).Inner);
                    end
                end
                nonlinearPart{j} = cell2mat(nonlinearPart{j});
            end
        end
        
        function [K,C,P,E,E2T,T,R,T2E] = OverlappingPreconditioner(this,t)
            assembly    = this.Assemblies_;
            nAssemblies = numel(assembly);
            assert(nAssemblies == 2);
            dp          = this.DomainPartitions;
            ind         = this.Index;
            
            Nt = [  ind.Local(1).Unknowns,...
                	ind.Local(2).Unknowns];
            
            Na = [  length(ind.Local(1).Boundary.Radius(2).Nodes) - 1,...
                 	length(ind.Local(2).Boundary.Radius(1).Nodes) - 1];
            
            K           = cell(1,nAssemblies);
            C           = cell(1,nAssemblies);
            P           = cell(1,nAssemblies);
            E           = cell(1,nAssemblies);
            E2T         = cell(1,nAssemblies);
            T           = cell(1,nAssemblies);
            R           = cell(1,nAssemblies);
            T2E         = cell(1,nAssemblies);
            
            [J1,J2,Jb]  = getDomainVectors(this);
            r           = [sparse(sum(J1|J2),sum(Jb)); speye(sum(Jb))]; 
            [E{:}]      = deal(r);
            [T{:}]      = deal(cell(1,numel(t)));
            [R{:}]      = deal(cell(1,numel(t)));
            
            %% Assemble Matrices
            for i = 1:2
                if i == 1
                    %% Indicies
                    I1 = dp(i).Domain(1).Unknowns;
                    I2 = dp(i).Domain(2).Unknowns;
                    Ib = I1 & I2;
                    I1 = ~Ib;
                    
                    %% Stiffness Matrix
                    K{i}      = cell(2,2);
                    K{i}{1,1} =    this.Stiffness.Reluctivity(i).El2El{1} ...
                                 + this.Stiffness.Reluctivity(i).El2El{2} ...
                                 + this.Stiffness.Reluctivity(i).Re2El    ...
                                 + this.Stiffness.Reluctivity(i).El2Re    ...
                                 + this.Stiffness.Reluctivity(i).Re2Re    ...
                                 + this.Stiffness.Reluctivity(i).Re2Cr    ...
                                 + this.Stiffness.Reluctivity(i).Cr2Re    ...
                                 + this.Stiffness.Reluctivity(i).Cr2Cr;
                    K{i}{1,1} = [K{i}{1,1}(I1,I1),K{i}{1,1}(I1,Ib);
                                 K{i}{1,1}(Ib,I1),K{i}{1,1}(Ib,Ib)];
                  	
                  	K{i}{2,2}        = real(this.Boundary.Radial.S(i+1).Inner * this.Boundary.Radial.F(i+1).Inner * this.Boundary.Radial.D(i+1).Inner);
                    Ia               = any(K{i}{2,2} ~= 0);
                    K{i}{2,2}(~Ia,:) = [];
                    K{i}{2,2}(:,~Ia) = [];
                    
                    K{i}{1,2} = real(this.Boundary.Radial.S(i).Outer   * this.Boundary.Radial.G(i+1).Inner * this.Boundary.Radial.D(i+1).Inner);
                  	K{i}{1,2} = [K{i}{1,2}(I1,Ia);K{i}{1,2}(Ib,Ia)];
                    
                    K{i}{2,1} = real(this.Boundary.Radial.S(i+1).Inner * this.Boundary.Radial.G(i).Outer   * this.Boundary.Radial.D(i).Outer);
                   	K{i}{2,1} = [K{i}{2,1}(Ia,I1),K{i}{2,1}(Ia,Ib)];
                    
                    %% Permutation Matrix (for full nonlinear contribution)
                    P{i}      = cell(2,2);
                    P{i}{1,1} = speye(Nt(1),Nt(1));
                    P{i}{1,2} = sparse(Nt(1),Nt(2));
                    P{i}{2,1} = sparse(Na(2),Nt(1));
                    P{i}{2,2} = sparse(Na(2),Nt(2));
                    
                    P{i}{1,1} = [   P{i}{1,1}(I1,:);
                                    P{i}{1,1}(Ib,:)];
                             
                  	%% Conductivity Matrix
                    C{i}{1,1} =   this.Mass.Conductivity(i).El2El{1} ...
                                + this.Mass.Conductivity(i).Re2El    ...
                                + this.Mass.Conductivity(i).El2Re    ...
                                + this.Mass.Conductivity(i).Re2Re;
                    C{i}{1,2} = sparse(Nt(1),Na(2));
                    C{i}{2,1} = sparse(Na(2),Nt(1));
                    C{i}{2,2} = sparse(Na(2),Na(2));
                    
                  	C{i}{1,1} = [C{i}{1,1}(I1,I1),C{i}{1,1}(I1,Ib);
                                 C{i}{1,1}(Ib,I1),C{i}{1,1}(Ib,Ib)];
                    
                    %% Reference Frame Transformation Matrices
                	a = this.Boundary.Radial.R(i).Outer - this.Boundary.Radial.R(i+1).Inner;
                    for j = 1:numel(t) 
                        r       = sparse(1:numel(a), 1:numel(a), exp(1i * a * t(j)));
                        T{i}{j} = real(this.Boundary.Radial.P(i+1).Inner * r * this.Boundary.Radial.D(i+1).Inner);
                        T{i}{j}(~Ia,:) = [];
                        T{i}{j}(:,~Ia) = [];
                        
                        R{i}{j} = real(this.Boundary.Radial.P(i+1).Inner * r' * this.Boundary.Radial.D(i+1).Inner);
                        R{i}{j}(~Ia,:) = [];
                        R{i}{j}(:,~Ia) = [];
                    end
                    
                    %% Restriction Matrices
                 	I1 = [dp(1).Domain(1).Unknowns | dp(1).Domain(2).Unknowns,...
                          dp(2).Domain(1).Unknowns & false];
                    Ib = [dp(1).Domain(1).Unknowns & dp(1).Domain(2).Unknowns,...
                          dp(2).Domain(1).Unknowns & dp(2).Domain(2).Unknowns & Ia];
                    I1 = I1 & ~Ib;
                    
                    E1      = sparse(sum(I1),sum(Jb));
                    Eb      = speye(length(Jb),length(Jb));
                    Eb      = Eb(:,Jb);
                    Eb      = Eb(Ib,:);
                    j       = (sum(Ib)-sum(Ia)+1):sum(Ib);
                    E2T{i}  = Eb(j,:);
                    Eb(j,:) = 0;
                    E{i}    = [E1;Eb];
                    T2E{i}  = [sparse(sum(I1)+sum(Ib)-sum(Ia),sum(Ia));
                               speye(sum(Ia),sum(Ia))];
                elseif i == 2
                    %% Indicies
                    I1 = dp(i).Domain(1).Unknowns;
                    I2 = dp(i).Domain(2).Unknowns;
                    Ib = I1 & I2;
                    I1 = ~Ib;
                    
                    %% Stiffness Matrix
                    K{i}      = cell(2,2);
                    K{i}{2,2} =    this.Stiffness.Reluctivity(i).El2El{1} ...
                                 + this.Stiffness.Reluctivity(i).El2El{2} ...
                                 + this.Stiffness.Reluctivity(i).Re2El    ...
                                 + this.Stiffness.Reluctivity(i).El2Re    ...
                                 + this.Stiffness.Reluctivity(i).Re2Re    ...
                                 + this.Stiffness.Reluctivity(i).Re2Cr    ...
                                 + this.Stiffness.Reluctivity(i).Cr2Re    ...
                                 + this.Stiffness.Reluctivity(i).Cr2Cr;
                    K{i}{2,2} = [K{i}{2,2}(I1,I1),K{i}{2,2}(I1,Ib);
                                 K{i}{2,2}(Ib,I1),K{i}{2,2}(Ib,Ib)];

                    K{i}{1,1}        = real(this.Boundary.Radial.S(i-1).Outer * this.Boundary.Radial.F(i-1).Outer * this.Boundary.Radial.D(i-1).Outer);
                    Ia               = any(K{i}{1,1} ~= 0);
                    K{i}{1,1}(~Ia,:) = [];
                    K{i}{1,1}(:,~Ia) = [];
                    
                    K{i}{2,1} = real(this.Boundary.Radial.S(i).Inner   * this.Boundary.Radial.G(i-1).Outer * this.Boundary.Radial.D(i-1).Outer);
                    K{i}{2,1} = [K{i}{2,1}(I1,Ia);K{i}{2,1}(Ib,Ia)];
                    
                    K{i}{1,2} = real(this.Boundary.Radial.S(i-1).Outer * this.Boundary.Radial.G(i).Inner   * this.Boundary.Radial.D(i).Inner);
                    K{i}{1,2} = [K{i}{1,2}(Ia,I1),K{i}{1,2}(Ia,Ib)];
                  	
                 	%% Permutation Matrix (for full nonlinear contribution)
                    P{i}      = cell(2,2);
                    P{i}{2,2} = speye(Nt(2),Nt(2));
                    P{i}{2,1} = sparse(Nt(2),Nt(1));
                    P{i}{1,2} = sparse(Na(1),Nt(2));
                    P{i}{1,1} = sparse(Na(1),Nt(1));
                    
                    P{i}{2,2} = [   P{i}{2,2}(I1,:);
                                    P{i}{2,2}(Ib,:)];
                    
                    %% Conductivity Matrix
                    C{i}{2,2} =   this.Mass.Conductivity(i).El2El{1} ...
                                + this.Mass.Conductivity(i).Re2El    ...
                                + this.Mass.Conductivity(i).El2Re    ...
                                + this.Mass.Conductivity(i).Re2Re;

                  	C{i}{2,2} = [C{i}{2,2}(I1,I1),C{i}{2,2}(I1,Ib);
                                 C{i}{2,2}(Ib,I1),C{i}{2,2}(Ib,Ib)];
                             
                    C{i}{1,2} = sparse(Na(1),Nt(2));
                    C{i}{2,1} = sparse(Nt(2),Na(1));
                    C{i}{1,1} = sparse(Na(1),Na(1));
                
                    %% Reference Frame Transformation Matrices
                    a = this.Boundary.Radial.R(i).Inner - this.Boundary.Radial.R(i-1).Outer;
                    for j = 1:numel(t)
                        r       = sparse(1:numel(a), 1:numel(a), exp(1i * a * t(j)));
                        T{i}{j} = real(this.Boundary.Radial.P(i-1).Outer * r * this.Boundary.Radial.D(i-1).Outer);
                      	T{i}{j}(~Ia,:) = [];
                        T{i}{j}(:,~Ia) = [];                        
                        
                        R{i}{j} = real(this.Boundary.Radial.P(i-1).Outer * r' * this.Boundary.Radial.D(i-1).Outer);
                      	R{i}{j}(~Ia,:) = [];
                        R{i}{j}(:,~Ia) = [];
                    end
                    
                    %% Restriction Matrices
                 	I1 = [dp(1).Domain(1).Unknowns & false,...
                          dp(2).Domain(1).Unknowns | dp(2).Domain(2).Unknowns];
                    Ib = [dp(1).Domain(1).Unknowns & dp(1).Domain(2).Unknowns & Ia,...
                          dp(2).Domain(1).Unknowns & dp(2).Domain(2).Unknowns];
                    I1 = I1 & ~Ib;
                    
                    E1      = sparse(sum(I1),sum(Jb));
                    Eb      = speye(length(Jb),length(Jb));
                    Eb      = Eb(:,Jb);
                    Eb      = Eb(Ib,:);
                    E{i}    = [0*Eb(1:sum(Ia),:);
                               E1;
                               Eb((sum(Ia)+1):end,:)];
                    E2T{i}  = Eb(1:sum(Ia),:);
                    T2E{i}  = [speye(sum(Ia),sum(Ia));
                               sparse(sum(I1)+sum(Ib)-sum(Ia),sum(Ia))];
                else
                    error('No Implementation');
                end
                   
                %% Convert to Matrix Format
                K{i} = cell2mat(K{i});
                P{i} = cell2mat(P{i});
                C{i} = cell2mat(C{i});
            end
        end
        
        function [nonlinearJacobian, nonlinearFunction] = G(this, t, x)
            nTimes   = numel(t);
            mesh     = this.Mesh_;
            assembly = [mesh.Assembly];
            index    = this.Index;
            curlE2N  = this.Jacobian.MagnetizationCurrent;
            curlN2E  = this.Jacobian.FluxDensity;
            
            nMesh     = numel(mesh);
            nUnknowns = index.Global(end).Unknowns;
            nRows     = zeros(nMesh,1);
            for i = 1:nMesh
                nRows(i) = index.Local(i).Unknowns;
            end
            
            nonlinearFunction      = mat2cell(sparse(nUnknowns,nTimes), nRows, nTimes);
            nonlinearJacobian      = cell(1,nTimes);
            [nonlinearJacobian{:}] = deal(mat2cell(sparse(nUnknowns,nUnknowns), nRows, nRows));

            for i = 1:nMesh
                materials = [assembly(i).Regions.Material];
                I         = index.Global(i).X;
                
                for k = 1:nTimes
                    Bx     = curlN2E(i).dBxdXz * x(I,k);
                    By     = curlN2E(i).dBydXz * x(I,k);
                    sizeB  = size(Bx);
                    Mx     = zeros(sizeB);
                    My     = zeros(sizeB);
                    dMxdBx = zeros(sizeB);
                    dMydBy = zeros(sizeB);
                    dMydBx = zeros(sizeB);
                    dMxdBy = zeros(sizeB);

                    for j = 1:numel(materials)
                        if ~materials(j).Linear
                            J = (mesh(i).ElementRegions == j);
                            [Mx(J) ,My(J), dMxdBx(J), dMydBy(J), dMydBx(J), dMxdBy(J)] = materials(j).vectorM(Bx(J), By(J));
    %                         %% Linear Model Debugging Code
    %                         Mx(J,:) = Bx(J,:)*(1-1/1000)/mu_o;
    %                         My(J,:) = By(J,:)*(1-1/1000)/mu_o;
    %                         dMxdBx(J,:) = (1-1/1000)/mu_o;
    %                         dMydBy(J,:) = (1-1/1000)/mu_o;
    %                         dMydBx(J,:) = 0;
    %                         dMxdBy(J,:) = 0;
                        end
                    end

                    J = 1:sizeB(1);
                    nonlinearJacobian{k}{i,i} ...
                        = + curlE2N(i).dIzdMx*( sparse(J,J,dMxdBx) * curlN2E(i).dBxdXz + sparse(J,J,dMxdBy) * curlN2E(i).dBydXz)...
                          + curlE2N(i).dIzdMy*( sparse(J,J,dMydBx) * curlN2E(i).dBxdXz + sparse(J,J,dMydBy) * curlN2E(i).dBydXz);

                    nonlinearFunction{i,k} = + curlE2N(i).dIzdMx * Mx  + curlE2N(i).dIzdMy * My;
                end
            end
            
            for k = 1:nTimes
                nonlinearJacobian{k} = cell2mat(nonlinearJacobian{k});
            end
                
            nonlinearFunction = cell2mat(nonlinearFunction);
        end
        
        function conductivityMatrix = C(this, ~, ~)
            assembly           = this.Assemblies_;
            nAssemblies        = numel(assembly);
            conductivityMatrix = cell(nAssemblies,nAssemblies);
            
            for i = 1:nAssemblies
                conductivityMatrix{i,i} =   this.Mass.Conductivity(i).El2El{1} ...
                                          + this.Mass.Conductivity(i).Re2El    ...
                                          + this.Mass.Conductivity(i).El2Re    ...
                                          + this.Mass.Conductivity(i).Re2Re;
            end
            
            conductivityMatrix = blkdiag(conductivityMatrix{:});
        end
        
        function exogenousFunction  = f(this, t, ~)
            nTimes   = numel(t);
            mesh     = this.Mesh_;
            assembly = [mesh.Assembly];
            
            nAssemblies       = numel(assembly);
            exogenousFunction = cell(nAssemblies,nTimes);
            for j = 1:nTimes
                for i = 1:nAssemblies
                    materials = [assembly(i).Regions.Material];
                    [~,Mx,My] = materials.vectorMr;
                    Mx        = Mx(mesh(i).ElementRegions);
                    My        = My(mesh(i).ElementRegions);
                    curl      = this.Jacobian.MagnetizationCurrent(i);

                    exogenousFunction{i,j} = (- curl.dIzdMx * Mx - curl.dIzdMy * My);

                    sources = assembly(i).Sources;
                    if numel(sources) > 0
                        exogenousFunction{i,j} = exogenousFunction{i,j} + (   this.Exogenous.Magnetic(i).Sr2El ...
                                                                            + this.Exogenous.Magnetic(i).Sr2Re ...
                                                                            + this.Exogenous.Magnetic(i).Sr2Cr) * sources.f(t(j));
                                                                                                                 
                    end
                end
            end
            exogenousFunction = cell2mat(exogenousFunction);
        end
        
        function [I1,I2,Ib] = getDomainVectors(this)
            dp = this.DomainPartitions;
            I1 = [dp(1).Domain(1).Unknowns,dp(2).Domain(1).Unknowns];
            I2 = [dp(1).Domain(2).Unknowns,dp(2).Domain(2).Unknowns];
            Ib = I1 & I2;
            I1 = I1 & ~Ib;
            I2 = I2 & ~Ib;
        end
    end
end