classdef DynamicMatrixFactory < MatrixFactory
    methods %Constructor
        function this = DynamicMatrixFactory(model)
            this = this@MatrixFactory(model);
        end
    end
    
    methods %Matrix and Vector Functions
        function linearMatrix = K(this, t, s) %Create stiffness matrix
            if nargin < 3
                s = 1;
            end
            
            assembly     = this.Assemblies;
            index        = this.Index.Local;
            nAssemblies  = numel(assembly);
            linearMatrix = cell(nAssemblies, nAssemblies);
            
            for i = 1:nAssemblies
                for j = 1:nAssemblies
                    linearMatrix{i,j} = sparse(index(i).Unknowns,index(j).Unknowns);
                end
            end
            
            for i = 1:nAssemblies
                linearMatrix{i,i} =   this.Stiffness.Reluctivity(i).Kff ...
                                    + this.Stiffness.Reluctivity(i).Kfc ...
                                    + this.Stiffness.Reluctivity(i).Kcf * s ...
                                    + this.Stiffness.Reluctivity(i).Kcc * s;
                                
                circuits = assembly(i).Circuits;
                for j = 1:numel(circuits);
                    linearMatrix{i,i} = linearMatrix{i,i} + circuits(j).K(t, s);
                end
                
                if i > 1
                    R = this.Boundary.Radial.R(i).Inner - this.Boundary.Radial.R(i-1).Outer;
                    R = sparse(1:numel(R), 1:numel(R), exp(1i * R * t));
                    
                    linearMatrix{i,i-1} ...
                        = real(this.Boundary.Radial.S(i).Inner * this.Boundary.Radial.G(i-1).Outer * R * this.Boundary.Radial.D(i-1).Outer);
                end
                
                if i < nAssemblies
                    R = this.Boundary.Radial.R(i).Outer - this.Boundary.Radial.R(i+1).Inner;
                    R = sparse(1:numel(R), 1:numel(R), exp(1i * R * t));
                    
                    linearMatrix{i,i+1} ...
                        = real(this.Boundary.Radial.S(i).Outer * this.Boundary.Radial.G(i+1).Inner * R * this.Boundary.Radial.D(i+1).Inner);
                end
            end
            
            for i = 1:nAssemblies
                for j = (i+1):nAssemblies
                    linearMatrix{i, j} = (linearMatrix{i,j} + linearMatrix{j,i}') / 2;
                    linearMatrix{j, i} = linearMatrix{i,j}';
                end
            end
            
            linearMatrix = cell2mat(linearMatrix);
        end
        
     	function [nonlinearJacobian, nonlinearFunction] = G(this, t, x, s)
            if nargin < 4
                s = 1;
            end
            
            mesh     = this.Mesh;
            assembly = [mesh.Assembly];
            index    = this.Index;
            curlE2N  = this.Jacobian.MagnetizationCurrent;
            curlN2E  = this.Jacobian.FluxDensity;
            
            nMesh = numel(mesh);
            nRows = zeros(nMesh,1);
            for i = 1:nMesh
                nRows(i) = index.Local(i).Unknowns;
            end
            
            nonlinearFunction = cell(nMesh,1);
            nonlinearJacobian = cell(nMesh,nMesh);
            for i = 1:nMesh
                nonlinearFunction{i} = sparse(nRows(i));
                for j = 1:nMesh
                    nonlinearJacobian{i,j} = sparse(nRows(i),nRows(j));
                end
            end

            for i = 1:nMesh
                materials   = [assembly(i).Regions.Material];
                isNonlinear = ~[materials.Linear];
                
                I = index.Global(i).X;
                
                Bx     = curlN2E(i).dBxdXz * x(I);
                By     = curlN2E(i).dBydXz * x(I);
                sizeB  = size(Bx);
                Mx     = zeros(sizeB);
                My     = zeros(sizeB);
                dMxdBx = zeros(sizeB);
                dMydBy = zeros(sizeB);
                dMydBx = zeros(sizeB);
                dMxdBy = zeros(sizeB);

                for j = 1:numel(materials)
                    if isNonlinear(j)
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
                nonlinearJacobian{i,i} ...
                    = + curlE2N(i).dIzdMx*( sparse(J,J,dMxdBx) * curlN2E(i).dBxdXz + sparse(J,J,dMxdBy) * curlN2E(i).dBydXz)...
                      + curlE2N(i).dIzdMy*( sparse(J,J,dMydBx) * curlN2E(i).dBxdXz + sparse(J,J,dMydBy) * curlN2E(i).dBydXz);

                nonlinearJacobian{i,i} = (nonlinearJacobian{i,i}+nonlinearJacobian{i,i}') / 2;
                 
                nonlinearFunction{i} = + curlE2N(i).dIzdMx * Mx  + curlE2N(i).dIzdMy * My;
                
                circuit = assembly(i).Circuits;
                for j = 1:numel(circuit)
                    if ~circuit(j).Linear
                        [Gc, gc] = circuit(j).g(t, x(I), s);
                        nonlinearJacobian{i,i} = nonlinearJacobian{i,i} + Gc;
                        nonlinearFunction{i,i} = nonlinearFunction{i,i} + gc;
                    end
                end
            end
            
            nonlinearJacobian = cell2mat(nonlinearJacobian);
            nonlinearFunction = cell2mat(nonlinearFunction);
        end
        
        function conductivityMatrix = C(this, t, h, s) %Create mass matrix
            if nargin < 3
                h = 1;
            end
            
            if nargin < 4
                s = h;
            end
            
            assembly           = this.Assemblies;
            nAssemblies        = numel(assembly);
            conductivityMatrix = cell(nAssemblies, nAssemblies);
           
            for i = 1:nAssemblies
                conductivityMatrix{i,i} =   this.Mass.Conductivity(i).Cff / h ...
                                          + this.Mass.Conductivity(i).Cfc / h...
                                          + this.Mass.Conductivity(i).Ccf * s / h...
                                          + this.Mass.Conductivity(i).Ccc * s / h;
                                      
                circuits = assembly(i).Circuits;
                for j = 1:numel(circuits)
                    conductivityMatrix{i,i} = conductivityMatrix{i,i} + circuits(j).C(t, h, s);
                end
            end

            conductivityMatrix = blkdiag(conductivityMatrix{:});
        end
        
    	function exogenousFunction = f(this, t, s)
            if nargin < 3
                s = 1;
            end
            
            
            mesh     = this.Mesh;
            assembly = [mesh.Assembly];
            
            nAssemblies       = numel(assembly);
            exogenousFunction = cell(nAssemblies,1);
            for i = 1:nAssemblies
                materials = [assembly(i).Regions.Material];
                [~,Mx,My] = materials.vectorMr;
                Mx        = Mx(mesh(i).ElementRegions);
                My        = My(mesh(i).ElementRegions);
                curl      = this.Jacobian.MagnetizationCurrent(i);
                
                exogenousFunction{i,1} = (- curl.dIzdMx * Mx - curl.dIzdMy * My);
                
               	circuits = assembly(i).Circuits;
                if numel(circuits) > 0
                    exogenousFunction{i,1} = exogenousFunction{i,1} + (   this.Exogenous.Magnetic(i).Ff     ...
                                                                        + this.Exogenous.Magnetic(i).Fc * s) * circuits.f(t, s);
                end
            end
            exogenousFunction = cell2mat(exogenousFunction);
        end
    end
end