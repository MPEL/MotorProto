classdef RotatingMachineMeshFactory < MeshFactory
    %RotatingMachineMeshFactory.m Creates a discretization of the input model
    %
    % DiscretizedGeometry methods:
    %
    %   
    % DiscretizedGeometry properties:
    %
    % See also MotorProto
    
    properties
        MaximumAirgapEdgeLength = [inf,inf];
    end
    
    properties (SetAccess = protected)
        RadialBoundaryEdges
        RadialBoundaryRadii

        PeriodicBoundaryNodes
    end
    
    methods
        function this = RotatingMachineMeshFactory(rma)
            if nargin > 0
                assert(isa(rma, 'RotatingMachineAssembly'), 'Assembly must be a RotatingMachineAssembly');
                assert(numel(rma) == 1, 'Assembly must be a scalar object');
                this.Assembly = rma;
            end
        end
        
        function curves = getAuxillaryBoundaries(this)
            r       = [this.Assembly.InnerRadius,this.Assembly.OuterRadius];
            r(1)    = [];
            r(end)  = [];
            nCurves = numel(r);
            curves  = Arc.empty(1,0);
            for i = 1:nCurves
                curves(i) = Geometry1D.draw('Arc', 'Radius', r(i), 'Angle', 2 * pi);
            end
        end
        
        function this = setBoundaryMaxEdgeLength(this)
            assembly = this.Assembly;
            r        = [assembly.InnerRadius, assembly.OuterRadius];
            mel      = this.MaximumAirgapEdgeLength;
            boundary = this.Boundaries;
            
            %% Find Arc boundaries centered at the origin
            Nbdry  = numel(boundary);
            search = false(1, Nbdry);
            for i = 1:Nbdry
                if all(boundary(i).Position == [0,0]) && isa(boundary(i), 'Arc')
                    search(i) = true;
                else
                    search(i) = false;
                end
            end
            
            %% Match Arc boundaries with inner/outer radius and set the maximum edge length
            J = find(search);
            for i = 1:2
                for j = J
                    if abs(boundary(j).Radius - r(i)) < sqrt(eps) * r(i)
                        this.Boundaries(j).MaxEdgeLength = mel(i);
                    end
                end
            end
        end
        
        function this = detectBoundaryPairs(this)
            %TODO - Clean up this method. Specifically, when numel(tRot) > 1'
            
            %% get boundary endpoints
            boundary    = this.Boundaries;
            nBoundaries = numel(boundary);
            
            x = zeros(2,nBoundaries);
            y = zeros(2,nBoundaries);
            for i = 1:nBoundaries
                x(:,i) = boundary(i).x([0 1]).';
                y(:,i) = boundary(i).y([0 1]).';
            end
            
            %% remove internal boundaries
            [~,onDomainHull]  = this.DomainHull.inOn(x(1,:),y(1,:));
            x                 = x(:,onDomainHull);
            y                 = y(:,onDomainHull);
            internalBoundary  = boundary(~onDomainHull);
            boundary          = boundary(onDomainHull);
            
            [~,onDomainHull]  = this.DomainHull.inOn(x(2,:),y(2,:));
            x                 = x(:,onDomainHull);
            y                 = y(:,onDomainHull);
            internalBoundary  = [internalBoundary, boundary(~onDomainHull)];
            boundary          = boundary(onDomainHull);
            
            %% remove radiall boundaries
            r     = [this.Assembly.InnerRadius, this.Assembly.OuterRadius];
            r_eps = max(r) * sqrt(eps);
            onRadialBoundary = false(1,numel(boundary));
            for i = 1:numel(boundary)
                if isa(boundary(i),'Arc')
                    if any(abs(r - boundary(i).Radius) < r_eps)
                        internalBoundary(end+1) = boundary(i);
                        onRadialBoundary(i)     = true;
                    end
                end
            end
            boundary(onRadialBoundary) = [];
            
            %% determine test points for intersections
            tRot = [this.DomainHull.Angle];
            tRot = unique(tRot);
            t    = atan2(y,x);
            r    = hypot(y,x);
            
            for k = 1:numel(tRot)
                tt = [t + tRot(k), t - tRot(k)];
                rr = [r, r];

                xTest = reshape(rr .* cos(tt), 1, []);
                yTest = reshape(rr .* sin(tt), 1, []);

                [~,onDomainHull] = this.DomainHull.inOn(xTest,yTest);
                xTest = xTest(onDomainHull);
                yTest = yTest(onDomainHull);

                %% split the boundaries when test points lie on an opposing boudnary
                nBoundaries = numel(boundary);
                for i = 1:nBoundaries
                    [S,On] = boundary(i).cart2s(xTest,yTest);
                    S         = S(On);
                    S(S == 0) = [];
                    S(S == 1) = [];
                    S         = unique(S);
                    if numel(S) > 0
                        newBoundary = boundary(i).split({S});
                        boundary(end+1:end+numel(newBoundary)) = newBoundary;
                    end
                end
            end
            
          	boundaryPairs  = zeros(2,nBoundaries);
            nBoundaryPairs = 0;
            for k = 1:numel(tRot)
                %% get the new boundary endpoints
                xNew = zeros(2,nBoundaries);
                yNew = zeros(2,nBoundaries);
                for i = 1:nBoundaries
                    xNew(:,i) = boundary(i).x([0 1]).';
                    yNew(:,i) = boundary(i).y([0 1]).';
                end

                rNew  = hypot(xNew,yNew);
                tNew  = atan2(yNew,xNew);

                tTest = tNew + tRot(k);
                xTest = rNew.*cos(tTest);
                yTest = rNew.*sin(tTest);

                r_eps = max(max(rNew)) * sqrt(eps);

                %% determine boundary pairs
                K = 1:nBoundaries;
                i = 1;
                while i <= nBoundaries
                    j = 1;
                    while j <= nBoundaries
                        dr = hypot(bsxfun(@minus, xTest(:,K(i)), xNew(:,K(j)).'), bsxfun(@minus, yTest(:,K(i)), yNew(:,K(j)).')) < r_eps;
                        if (dr(1,1) && dr(2,2)) || (dr(1,2) && dr(2,1))
                            nBoundaryPairs = nBoundaryPairs + 1;
                            boundaryPairs(:,nBoundaryPairs) = [K(i);K(j)];
                        end
                        j = j + 1;
                    end
                    i = i + 1;
                end
            end
            boundaryPairs(:,nBoundaryPairs+1:end) = [];
            
            this.BoundaryPairs = boundaryPairs;
            this.Boundaries    = [boundary,internalBoundary];
        end
        
        function this = detectEdgePairs(this)
            boundaryPairs  = this.BoundaryPairs;
            nBoundaryPairs = numel(boundaryPairs) / 2;
            
            constEdges     = this.ConstrainedEdges;
            constEdgeBound = this.ConstrainedEdgeBoundaries;
            constEdgePairs = zeros(2,0);
            
            x = this.X(constEdges);
            y = this.Y(constEdges);
            
            for i = 1:nBoundaryPairs
                boundaryPairI = (constEdgeBound == boundaryPairs(1,i));
                boundaryPairJ = (constEdgeBound == boundaryPairs(2,i));
                
                nPairs = sum(boundaryPairI);
                
                xi = mean(x(:,boundaryPairI));
                yi = mean(y(:,boundaryPairI));
                ri = hypot(xi,yi);
                ti = atan2(yi,xi);
                ti = unwrap(ti);
                
                xj = mean(x(:,boundaryPairJ));
                yj = mean(y(:,boundaryPairJ));
                rj = hypot(xj,yj);
                tj = atan2(yj,xj);
                tj = unwrap(tj);
                
                r_eps = max(max(ri),max(rj)) * sqrt(eps);
                t_eps = 2 * pi * sqrt(eps);
                
                %% detect the rotation angle
                dt      = bsxfun(@minus,ti.',tj);
              	dt      = round(dt / t_eps) * t_eps;
              	dt      = double(single(dt));
              	t       = mode(dt(:));

                sameRadius        = abs(bsxfun(@minus,ri.',rj)) < r_eps;
                sameRotationAngle = abs(dt - t) < t_eps;
                
                %% find the edge pairs
                [edgePairI,edgePairJ] = find(sameRadius & sameRotationAngle);
                
                if numel(edgePairI) > nPairs
                    error('MotorProto:MeshFacotry', 'Excessive number of edge pairs found. This is likely due to a corner case which is not handled correctly');
                elseif numel(edgePairI) < nPairs
                    error('MotorProto:MeshFactory', 'Not enough edge pairs found');
                end
                
                boundaryPairI = find(boundaryPairI);
                boundaryPairJ = find(boundaryPairJ);
                
                boundaryPairI = boundaryPairI(edgePairI);
                boundaryPairJ = boundaryPairJ(edgePairJ);
                
                constEdgePairs(1,end+1:end+nPairs) = boundaryPairI;
                constEdgePairs(2,end-nPairs+1:end) = boundaryPairJ;
            end
            
            this.ConstrainedEdgePairs = constEdgePairs;
        end
        
        function this = compileBoundaryData(this)
            assembly = this.Assembly;

            %% get radial boundary pairs
            rbRadii = [assembly.InnerRadius,assembly.OuterRadius];
            rbRadii = sort(rbRadii);
            
            %% get boundary edges
            edges = this.BoundaryEdges;
            x     = this.X(edges);
            y     = this.Y(edges);
            r     = hypot(x,y);
            
            isRadEdge    =(abs(r(1,:) - r(2,:)) < (sqrt(eps) * mean(r)));
            radEdgeRadii = mean(r(:,isRadEdge));
            radEdges     = edges(:, isRadEdge);
            
            %% associate boundary edges/radii
            rbEdges = cell(1,2);
            
            k1           = abs(radEdgeRadii - rbRadii(1,1))...
                                < sqrt(eps) * rbRadii(1,1);
            rbEdges{1,1} = radEdges(:,k1);
            
            k2           = abs(radEdgeRadii - rbRadii(1,2))...
                                < sqrt(eps) * rbRadii(1,2);
            rbEdges{1,2} = radEdges(:,k2);

            this.RadialBoundaryEdges = rbEdges;
            this.RadialBoundaryRadii = rbRadii;
            
            %% get periodic boundary nodes
            cePairs = this.ConstrainedEdgePairs;
            
            if ~isempty(cePairs)
                cEdges1 = this.ConstrainedEdges(:,cePairs(1,:));
                cEdges2 = this.ConstrainedEdges(:,cePairs(2,:));

                x = this.X;
                y = this.Y;

                x1 = [x(cEdges1(1,:)); x(cEdges1(2,:))];
                y1 = [y(cEdges1(1,:)); y(cEdges1(2,:))];
                r1 = hypot(x1,y1);
                t1 = atan2(y1,x1);
                t1 = unwrap(t1);
                
                x2 = [x(cEdges2(1,:)); x(cEdges2(2,:))];
                y2 = [y(cEdges2(1,:)); y(cEdges2(2,:))];
                r2 = hypot(x2,y2);
                t2 = atan2(y2,x2);
                t2 = unwrap(t2);

                r_eps = max(max(max(r1)), max(max(r2))) * sqrt(eps);
                t_eps = 2 * pi * sqrt(eps);

                dr = all(abs(r1 - r2) < r_eps);
                cEdges2([1,2], ~dr) = cEdges2([2,1], ~dr);
                r2([1,2],~dr)       = r2([2,1], ~dr);
                t2([1,2],~dr)       = t2([2,1], ~dr);

                assert(all(all(abs(r1 - r2) < r_eps)));

                dt = abs((t1(1,:) - t1(2,:)) - (t2(1,:) - t2(2,:))) < t_eps;
                cEdges2([1,2], ~dt) = cEdges2([2,1], ~dt);
                r2([1,2],~dt)       = r2([2,1], ~dt);
                t2([1,2],~dt)       = t2([2,1], ~dt);

                assert(all(all(abs(r1 - r2) < r_eps)));
                assert(all(abs((t1(1,:) - t1(2,:)) - (t2(1,:) - t2(2,:))) < t_eps));

                pbNodes = [cEdges1(1,:),cEdges1(2,:);...
                           cEdges2(1,:),cEdges2(2,:)].';
                pbNodes = unique(pbNodes,'rows').';
            else
                pbNodes = [];
            end
               
            this.PeriodicBoundaryNodes = pbNodes;
        end
    end
end