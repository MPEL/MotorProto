classdef MeshFactory < matlab.mixin.Copyable
    properties (SetAccess = protected)
        Assembly = Stator.empty(1,0);
    end
           
    properties (SetAccess = protected, Dependent)
        Components
        Regions
        Sources
        Geometry
        Materials
        GeometryMirrorSymmetry
        GeometryFrequency
        DomainHull
    end
    
    properties (SetAccess = protected)
        Boundaries
        BoundaryPairs
        BackgroundTriangulation
        
        X
        Y
        IsFixedNode
        IsConstrainedNode
        IsBoundingBoxNode
        
        Elements
        ElementAreas
        ElementIncenters
        ElementInradii
        ElementCircumcenters
        ElementCircumradii
        
        ElementAngles
        ElementRegions
        
        ConstrainedEdges
        ConstrainedEdgeBoundaries
        ConstrainedEdgeParameters
        ConstrainedEdgePairs

        BoundaryEdges
        BoundaryElements

        ElementRefinementQueue
        EdgeRefinementQueue
        VertexRemovalQueue
    end
    
    properties
        UseUniformGrid     = false;
        UniformGridType    = 'rectangular';
        ElementOrder       = 1;
        MinimumElementSize = 0;
        MaximumElementSize = inf;
    end
    
    methods
        %% Constructor
        function this = MeshFactory
        end
        
        %% Getters
        function value = get.GeometryMirrorSymmetry(this)
            value = this.Assembly.GeometryMirrorSymmetry;
        end
        
        function value = get.GeometryFrequency(this)
            value = this.Assembly.GeometryFrequency;
        end
        
        function value = get.Components(this)
            value = this.Assembly.Components;
        end
        
        function value = get.Regions(this)
            value = this.Assembly.Regions;
        end
        
        function value = get.Sources(this)
            value = this.Assembly.Sources;
        end
        
        function value = get.Geometry(this)
            value = this.Assembly.Geometry;
        end
        
        function value = get.Materials(this)
            value = [this.Assembly.Regions.Material];
        end
        
        function value = get.DomainHull(this)
            value = this.Assembly.DomainHull;
        end
        
        %% Other
        function [In,On] = inDomainHull(this,x,y)
            [In,On] = this.DomainHull.inOn(x,y);
            In      = In.';
            On      = On.';
        end
        
        function [x,y,r] = getBoundingBall(this)
            [x,y,r] = this.Boundaries.getBoundingBall;
            
            dr     = hypot(bsxfun(@minus,x,x.'),bsxfun(@minus,y,y.'));
            [dr,i] = max(tril(dr,-1));
            [dr,j] = max(dr);
            i      = i(j);
            
            dx = x(j) - x(i);
            dy = y(j) - y(i);
            a  = atan2(dy,dx);
            
            x = (x(i) + x(j)) / 2 + (r(j) - r(i)) * cos(a) / 2;
            y = (y(i) + y(j)) / 2 + (r(j) - r(i)) * sin(a) / 2;
            r = (dr + r(i) + r(j)) / 2;
        end

        function I = inBoundingBall(this,xIn,yIn)
            [x,y,r] = getBoundingBall(this);
            dr      = hypot(xIn - x, yIn - y);
            I       = (dr < r * (1 + sqrt(eps)));
        end
        
        function this = build(this)
            %build - Generates a mesh from the current input geometry
            %	D = mesh(D) generates a mesh from the regions defined in the
            %	Planes property.
            %
            % See also DiscretizedGeometry
            
            %TODO - There is probably a more elegent way to handle the airgap situation
            %TODO - Add in auxillary boundaries from adjacent annuli
            
            nThis = numel(this);
            for i = 1:nThis
                %% determine the initial peicewise linear system
                this(i) = getBoundaries(this(i));
                
                this(i) = discretizeBoundaries(this(i));

                %% refine the initial pls
                this(i) = addEdgesToQueue(this(i));

                while ~isempty(this(i).EdgeRefinementQueue)
                    this(i) = removeVertices(this(i));

                    this(i) = insertEdgeMidpoints(this(i));

                    this(i) = addEdgesToQueue(this(i));
                end

                %% if using a uniform grid, insert it now
                if this(i).UseUniformGrid
                    this(i) = addUniformGrid(this(i));
                end

                %% perform initial triangulation and smoothing
                this(i) = addBoundingBox(this(i));

                this(i) = updateTriangulation(this(i));

                for j = 1:2
                    this(i) = smoothTriangulation(this(i));

                    this(i) = updateTriangulation(this(i));
                end

                %% refine the mesh for quality and size
                this(i) = addElementsToQueue(this(i));

                while ~isempty(this(i).ElementRefinementQueue)
                    while ~isempty(this(i).ElementRefinementQueue);
                        this(i) = insertElementCircumcenters(this(i));

                        this(i) = removeVertices(this(i));

                        this(i) = insertEdgeMidpoints(this(i));

                        this(i) = updateTriangulation(this(i));

                        this(i) = smoothTriangulation(this(i));

                        this(i) = updateTriangulation(this(i));

                        this(i) = addElementsToQueue(this(i));
                    end

                    %% if the queue is empty perform a few extra smoothing
                    %  operations to make sure the process has converged
                    for j = 1:2
                        this(i) = smoothTriangulation(this(i));

                        this(i) = updateTriangulation(this(i));
                    end

                    this(i) = addElementsToQueue(this(i));
                end

                %% finalize the mesh
                this(i) = removeBoundingBox(this(i),false);
                
                this(i) = doPostProcessing(this(i));

                this(i) = increaseElementOrder(this(i));
                
                this(i) = compileBoundaryData(this(i));
            end
        end
        
        function this = getBoundaries(this)
            %% Returns non-intersecting material boundaries
            regions   	    = this.Regions;
            geometry        = [regions.Geometry];
            curves          = [geometry.Curves];
            this.Boundaries = makeUnique(curves);
            this            = detectBoundaryPairs(this);
            this            = setBoundaryMaxEdgeLength(this);
        end
        
        function this = discretizeBoundaries(this)
            %% Takes non-intersecting material boundaries and produces a set of
            %  approximating edges, with maximum length determined by curvature
            %  and specified maximum element size
            
            curves  = this.Boundaries;
            nCurves = numel(curves);
            cEdges  = curves.MinEdgeNumber;
            
            if this.UseUniformGrid
                cLength = curves.length / (this.MaximumElementSize * sqrt(3) * 7 / (sqrt(3) * 2 + sqrt(2)*3));
            else
                cLength = curves.length / (this.MaximumElementSize * sqrt(3));
            end
            
            cEdges = max(cEdges, ceil(cLength)+1);
            cNodes = cEdges + 1;
            
            nEdges = sum(cEdges);
            nNodes = sum(cNodes);
            
            x = zeros(1,nNodes);
            y = zeros(1,nNodes);

            edges     = zeros(2,nEdges);
            edgeParam = zeros(2,nEdges);
            edgeBound = zeros(1,nEdges);

            nodeIsFixed = false(1,nNodes);
            nodeIsConst = true(1,nNodes);
            
            iEdge = 0;
            jNode = 0;
            for kCurve = 1:nCurves
                IEdge = iEdge + (1:cEdges(kCurve));
                JNode = jNode + (1:cNodes(kCurve));
                SNode = linspace(0,1,cNodes(kCurve));
                
                edges(1,IEdge)     = JNode(1:end-1);
                edges(2,IEdge)     = JNode(2:end);
                edgeParam(1,IEdge) = SNode(1:end-1);
                edgeParam(2,IEdge) = SNode(2:end);
                edgeBound(IEdge)   = kCurve;
                
                nodeIsFixed(JNode([1 end])) = true;
                nodeIsConst(JNode([1 end])) = false;
                
                x(JNode) = curves(kCurve).x(SNode);
                y(JNode) = curves(kCurve).y(SNode);
                
                iEdge = IEdge(end);
                jNode = JNode(end);
            end
            
            %% find duplicate nodes
            leps   = min(curves.length) * sqrt(eps);
           	dr     = hypot(bsxfun(@minus,x,x.'),bsxfun(@minus,y,y.'));
            isSame = tril(dr < leps,-1);
            
           	%% remove duplicates, renumber the nodes in a consistent manner
            for jNode = 1:nNodes;
                sameNode           = isSame(:,jNode);
                isSame(:,sameNode) = false;
                isSame(:,jNode)    = sameNode;
            end
            [renumber,keep] = find(isSame);
            
            for jNode = 1:numel(renumber)
                iEdge        = (edges == renumber(jNode));
                edges(iEdge) = keep(jNode);
                
                x(renumber(jNode)) = [];
                y(renumber(jNode)) = [];
                nodeIsFixed(renumber(jNode)) = [];
                nodeIsConst(renumber(jNode)) = [];
                
                k = (keep     > renumber(jNode));
                r = (renumber > renumber(jNode));
                c = (edges    > renumber(jNode));
                
                keep(k)     = keep(k)     - 1;
                renumber(r) = renumber(r) - 1;
                edges(c)    = edges(c)    - 1;
                nNodes      = nNodes      - 1;
            end
            
            %% sort edges
            [edges,I] = sort(edges);
            for iEdge = 1:nEdges
                edgeParam(:,iEdge) = edgeParam(I(:,iEdge),iEdge);
            end
            
            [edges,I]      = sortrows(edges.');
            edges          = edges.';
            edgeParam(1,:) = edgeParam(1,I);
            edgeParam(2,:) = edgeParam(2,I);
            edgeBound      = edgeBound(I);
            
            %% assign outputs
            this.X = x;
            this.Y = y;
            
            this.ConstrainedEdges          = edges;
            this.ConstrainedEdgeParameters = edgeParam;
            this.ConstrainedEdgeBoundaries = edgeBound;

            this.IsFixedNode       = nodeIsFixed;
            this.IsConstrainedNode = nodeIsConst;
            this.IsBoundingBoxNode = false(1,nNodes);
            
            this = detectEdgePairs(this);
        end
        
        function this = addEdgesToQueue(this)
            %% Split all discrete boundary edges until none are encroached,
            %  based on the first stage of Ruperts algorithm for creating 
            %  constrained Delaunay Triangulations
            
            %TODO - This method can result in a non-terminating loop if two edges share a vertex and make an accute angle
            
            x      = this.X;
            y      = this.Y;
            cEdges = this.ConstrainedEdges;
            
            xe  = x(cEdges);
            ye  = y(cEdges);
            xem = mean(xe);
            yem = mean(ye);
            dre = hypot(diff(xe),diff(ye)) / 2;
            
            edgeHasPointInDiametralBall = false(1,numel(cEdges)/2);
            vertexIsInDiametralBall     = false(1,numel(x));
            for i = 1:(numel(cEdges)/2)
                dr = hypot(xem(i) - x, yem(i) - y);
                I  = (dr < dre(i) * (1 - sqrt(eps)));
                
                edgeHasPointInDiametralBall(i) = any(I);
                vertexIsInDiametralBall(I)     = true;
            end
            
            if any(edgeHasPointInDiametralBall)
                this.EdgeRefinementQueue = find(edgeHasPointInDiametralBall);

                removeVertex = vertexIsInDiametralBall & ~this.IsConstrainedNode & ~this.IsFixedNode;

                this.VertexRemovalQueue = find(removeVertex);
            end
        end
        
        function this = insertEdgeMidpoints(this)
            erQueue = this.EdgeRefinementQueue;
          	cePairs = this.ConstrainedEdgePairs;
            nQueue  = numel(erQueue);
            
            queuePair = false(size(cePairs));
            for i = 1:nQueue
                hasPair   = (cePairs == erQueue(i));
                queuePair = queuePair | hasPair | flipud(hasPair);
            end
            
            erQueue      = unique([erQueue,cePairs(queuePair).']);
            nQueue       = numel(erQueue);
            
            cEdges       = this.ConstrainedEdges;
            ceBoundaries = this.ConstrainedEdgeBoundaries;
            ceParameters = this.ConstrainedEdgeParameters;
            
            boundaries   = this.Boundaries;
            
            nNodes = length(this.X);
            xNew   = zeros(1,nQueue);
            yNew   = zeros(1,nQueue);
            eNew   = zeros(2,nQueue);
            sNew   = NaN(2,nQueue);
            bNew   = NaN(1,nQueue);
            
            for i = 1:nQueue
                j = erQueue(i);
                
                s         = (ceParameters(1,j) + ceParameters(2,j)) / 2;
                b         = ceBoundaries(j);
                bNew(i)   = b;
                xNew(i)   = boundaries(b).x(s);
                yNew(i)   = boundaries(b).y(s);
                
                eNew(2,i)   = nNodes + i;
                eNew(1,i)   = cEdges(2,j);
                cEdges(2,j) = eNew(2,i);
                
                sNew(2,i) = s;
                sNew(1,i) = ceParameters(2,j);
                ceParameters(2,j) = sNew(2,i);
            end
            
            this.IsConstrainedNode(end+1:end+nQueue) = true;
            this.IsFixedNode(end+1:end+nQueue)       = false;
            this.IsBoundingBoxNode(end+1:end+nQueue) = false;
            
            this.ConstrainedEdges          = [cEdges,eNew];
            this.ConstrainedEdgeParameters = [ceParameters,sNew];
            this.ConstrainedEdgeBoundaries(end+1:end+nQueue) = bNew;
            
            this.X(end+1:end+nQueue) = xNew;
            this.Y(end+1:end+nQueue) = yNew;
            
            this = detectEdgePairs(this);
            
            this.EdgeRefinementQueue = [];
        end
        
        function this = addUniformGrid(this)
            %% get constrained edge data
        	xe = this.X(this.ConstrainedEdges);
            dx = xe(1,:) - xe(2,:);
            
            ye = this.Y(this.ConstrainedEdges);
            dy = ye(1,:) - ye(2,:);
            
            re = hypot(dx,dy);
            
            %% get the optimal edge length for creating a uniform grid of 
            %  equilateral triangles, given that some elements inserted near
            %  constrained boundaries will need to be deformed
            lmax           = max(re);
            [xbb,ybb,rmax] = this.getBoundingBall;
            
            switch lower(this.UniformGridType)
                case 'rectangular'
                    nx = ceil(2 * rmax / lmax) + 2;
                    ny = ceil(2 * rmax / (lmax * sqrt(3))) + 2;

                    x1 = repmat((-nx / 2:nx / 2) * lmax,ny + 1,1);
                    x2 = x1 + lmax / 2;

                    y1 = repmat((-ny / 2:ny / 2) * lmax * sqrt(3),1,nx + 1);
                    y2 = y1 + lmax * sqrt(3) / 2;

                    x = [reshape(x1,1,[]), reshape(x2,1,[])] + xbb;
                    y = [y1,y2] + ybb;
            
                case 'polar'
                    r = lmax;
                    x = 0;
                    y = 0;
                    while r < rmax
                        n      = ceil(2 * pi * r / lmax);
                        a      = linspace(0,2*pi,n+1);
                        a(end) = [];

                       x(end+1:end+n) = r * cos(a);
                       y(end+1:end+n) = r * sin(a);
                       r = r + lmax;
                    end
                otherwise
                    error('MotorProto:MeshFactory:unknownOption',...
                            'Unknown grid type "%s"',this.UniformGridType);
            end
            
            %% remove nodes outside the domain
            i     = inDomainHull(this,x,y);
            x(~i) = [];
            y(~i) = [];
            
            %% remove nodes which will result in triangles with smallest angle
            %  less than 30 degrees near constrained edges
            dr   = hypot(bsxfun(@minus,x,xe(1,:).'),bsxfun(@minus,y,ye(1,:).'));
            i    = any(bsxfun(@lt,dr,re.' * (1+sqrt(eps)) / sqrt(3)));
            x(i) = [];
            y(i) = [];
            
            dr   = hypot(bsxfun(@minus,x,xe(2,:).'),bsxfun(@minus,y,ye(2,:).'));
            i    = any(bsxfun(@lt,dr,re.' * (1+sqrt(eps)) / sqrt(3)));
            x(i) = [];
            y(i) = [];
            
            nNodes = sum(~i);
            
            %% assign outputs
            this.X(end+1:end+nNodes) = x;
            this.Y(end+1:end+nNodes) = y;
            
            this.IsConstrainedNode(end+1:end+nNodes) = false;
            this.IsFixedNode(end+1:end+nNodes)       = false;
            this.IsBoundingBoxNode(end+1:end+nNodes) = false;
        end
        
        function this = updateTriangulation(this)
            hasBB = any(this.IsBoundingBoxNode);
            if ~hasBB
                this.addBoundingBox(false);
            end
            
            updateBackgroundTriangulation(this);
            
            this.Elements = this.BackgroundTriangulation.Triangulation.';
            
            x           = this.X(this.Elements);
            y           = this.Y(this.Elements);
            edgeLength2 = [(x(1,:)-x(2,:)).^2;...
                           (x(2,:)-x(3,:)).^2;...
                           (x(3,:)-x(1,:)).^2]...
                         +[(y(1,:)-y(2,:)).^2;...
                           (y(2,:)-y(3,:)).^2;...
                           (y(3,:)-y(1,:)).^2];
                            
            edgeLength  = sqrt(edgeLength2);
            elementAngle = [(edgeLength2(1,:)-edgeLength2(2,:)-edgeLength2(3,:));...
                            (edgeLength2(2,:)-edgeLength2(3,:)-edgeLength2(1,:));
                            (edgeLength2(3,:)-edgeLength2(1,:)-edgeLength2(2,:))];
                        
            elementAngle = elementAngle ./ ...
                            [edgeLength(2,:).*edgeLength(3,:);...
                             edgeLength(3,:).*edgeLength(1,:);...
                             edgeLength(1,:).*edgeLength(2,:)] / (-2);
            
            this.ElementAngles = elementAngle;

            [this.ElementIncenters,...
               	this.ElementInradii] = this.BackgroundTriangulation.incenters;
            this.ElementIncenters    = this.ElementIncenters.';
            this.ElementInradii      = this.ElementInradii.';
            
            [this.ElementCircumcenters,...
                this.ElementCircumradii] = this.BackgroundTriangulation.circumcenters;
            this.ElementCircumcenters    = this.ElementCircumcenters.';
            this.ElementCircumradii      = this.ElementCircumradii.';

            if ~hasBB
                this.removeBoundingBox(false);
            end
        end
        
        function this = addBoundingBox(this,doUpdate)
            if ~any(this.IsBoundingBoxNode)
                if nargin == 1
                    doUpdate = true;
                end
                
                [xc,yc,rc] = this.getBoundingBall;
                nNodes = 8;
                r      = 2 * rc;
                a      = linspace(0,2*pi,nNodes + 1);
                a(end) = [];
                x      = r * cos(a) + xc;
                y      = r * sin(a) + yc;
                
                this.X(end+1:end+nNodes)                 = x;
                this.Y(end+1:end+nNodes)                 = y;
                
                this.IsFixedNode(end+1:end+nNodes)       = true;
                this.IsConstrainedNode(end+1:end+nNodes) = false;
                this.IsBoundingBoxNode(end+1:end+nNodes) = true;
                if doUpdate
                    updateTriangulation(this);
                end
            end
        end
        
        function this = removeBoundingBox(this,update)
            if any(this.IsBoundingBoxNode)
                if nargin == 1
                    update = true;
                end

                %% remove bounding box nodes
                isBBNode = this.IsBoundingBoxNode;
                i        = find(isBBNode);              
             	isBBEl   = any(  this.Elements >= min(i)...
                               & this.Elements <= max(i));
                                        
                this.X(isBBNode)                 = [];
                this.Y(isBBNode)                 = [];
                this.IsFixedNode(isBBNode)       = [];
                this.IsConstrainedNode(isBBNode) = [];
                this.IsBoundingBoxNode(isBBNode) = [];
                
                this.Elements(:,isBBEl)           	= [];
                this.ElementIncenters(:,isBBEl)   	= [];
                this.ElementCircumcenters(:,isBBEl)	= [];
                this.ElementInradii(isBBEl)       	= [];
                this.ElementCircumradii(isBBEl)   	= [];
                this.ElementAngles(:,isBBEl)     	= [];
                
                %% renumber nodes
                k = max(i);
                n = length(i);

                this.Elements(this.Elements > k) = ...
                    this.Elements(this.Elements > k) - n;
                
                this.ConstrainedEdges(this.ConstrainedEdges > k) = ...
                    this.ConstrainedEdges(this.ConstrainedEdges > k) - n;
                
                if update
                    updateTriangulation(this);
                end
            end
        end
        
        function this = addElementsToQueue(this)
            circumcenters = this.ElementCircumcenters;
            circumradii   = this.ElementCircumradii;
            incenters     = this.ElementIncenters;
            inradii       = this.ElementInradii;
            
            refineable = this.ElementAngles > sqrt(3) / 2;
            refineable = any(refineable);
            refineable = refineable | (circumradii > this.MaximumElementSize);
            refineable = refineable & (inradii     > this.MinimumElementSize);
            refineable = refineable & inDomainHull(this,incenters(1,:),incenters(2,:));
            
            
            nElements = length(this.Elements);
            queue     = zeros(1,nElements);
            
            elementID = find(refineable);
            k         = 1;
            while any(refineable)
                %% get largest element that can be refined
                [~, i]   = max(circumradii(refineable));
               	j        = elementID(i);
                queue(k) = j;
                
                %% remove all elements whose circumcenter is within
                %  the largest element's circumscribed circle
                dc = sqrt(sum(bsxfun(@minus, circumcenters(:,refineable), circumcenters(:,j)).^2));

                l             = dc < circumradii(j) * (1 + sqrt(eps));
                f             = elementID(l);
                refineable(f) = false;
                elementID(l)  = [];                
               
                %% get the element containing the largest element's circumcenter
                dc  = hypot(incenters(1,refineable) - circumcenters(1,j), incenters(2,refineable) - circumcenters(2,j));
                jEl = find(any(dc < inradii(refineable) * (1 + sqrt(eps))));
                
                %% remove all elements whose circumscribed circle intersects
                % with these elements inscribed circles
                if ~isempty(jEl)
                    dr = bsxfun(@plus,circumradii(refineable),inradii(jEl).');
                    dc = hypot(bsxfun(@minus,circumcenters(1,refineable),incenters(1,jEl).'),...
                               bsxfun(@minus,circumcenters(2,refineable),incenters(2,jEl).'));

                    l = any(dc < dr * (1 + sqrt(eps)),1);
                    f = elementID(l);
                    refineable(f) = false;
                    elementID(l)  = [];   
                end
                
                %% procede to next
                k  = k + 1;
            end
            queue(k:end) = [];
            this.ElementRefinementQueue = queue;
        end
        
        function this = insertElementCircumcenters(this)
            x             = this.X;
            y             = this.Y;
            nNodes        = length(x);

            erQueue       = this.ElementRefinementQueue;

            incenters     = this.ElementIncenters(:,erQueue);
            circumcenters = this.ElementCircumcenters(:,erQueue);
            
            xe            = [incenters(1,:); circumcenters(1,:)];
            ye            = [incenters(2,:); circumcenters(2,:)];
            
            cEdges        = this.ConstrainedEdges;
            
            xc            = this.X(cEdges);
            xcc           = (xc(1,:) + xc(2,:)) / 2;
            yc            = this.Y(cEdges);
            ycc           = (yc(1,:) + yc(2,:)) / 2;
            rc            = hypot(xc(1,:) - xc(2,:),yc(1,:) - yc(2,:)) / 2;
            
            nNew                = length(erQueue);
            xNew                = zeros(1, nNew);
            yNew                = zeros(1, nNew);
            encroachedEdges     = zeros(1, nNew);
            encroachingVertices = false(1, nNodes);
            successfulInsertion = false(1, nNew);
            
            [blocksLOS, xint, yint] = this.findIntersectingEdges(xe, ye, xc, yc);
            for i = 1:nNew
                if ~isempty(blocksLOS{i})
                    %% find the closest edge that blocks LOS
                    successfulInsertion(i) = false;
                    
                    d2ic   = hypot(xint{i} - incenters(1,i), yint{i} - incenters(2,i));
                    [~,k]  = min(d2ic);
                    k      = blocksLOS{i}(k);
                    encroachedEdges(i) = k;
                    
                    %% remove all nodes in the edge diametral ball
                    d2ec                = hypot(x - xcc(k), y - ycc(k));
                    encroachingVertices = encroachingVertices | (d2ec < rc(k) * (1 - sqrt(eps)));
                    
                else
                    %% To avoid degenerate triangles, shift the vertex
                    %  toward the incenter of the triangle.               
                    successfulInsertion(i) = true;
                    
                    xNew(i) = circumcenters(1,i) * (1-sqrt(eps)) + incenters(1,i) * sqrt(eps);
                    yNew(i) = circumcenters(2,i) * (1-sqrt(eps)) + incenters(2,i) * sqrt(eps);
                end
            end
            removeVertices = encroachingVertices & ~this.IsFixedNode & ~this.IsConstrainedNode;
            
            nNew = sum(successfulInsertion);
            this.X(end+1:end+nNew)                 = xNew(successfulInsertion);
            this.Y(end+1:end+nNew)                 = yNew(successfulInsertion);
            this.IsConstrainedNode(end+1:end+nNew) = false;
            this.IsFixedNode(end+1:end+nNew)       = false;
            this.IsBoundingBoxNode(end+1:end+nNew) = false;
            
            this.ElementRefinementQueue = [];
            this.EdgeRefinementQueue    = encroachedEdges(~successfulInsertion);
            this.VertexRemovalQueue     = find(removeVertices);
        end
        
        function this = smoothTriangulation(this)
            x      = this.X;
            y      = this.Y;
            el     = this.Elements;
            nNodes = max(max(el));
            nEl    = length(el);
            
            i = [el(1,:),el(2,:),el(3,:),el(1,:),el(2,:),el(3,:),el(1,:),el(2,:),el(3,:)];
            j = [el(1,:),el(2,:),el(3,:),el(2,:),el(3,:),el(1,:),el(3,:),el(1,:),el(2,:)];
            
            i = [i,i + nNodes];
            j = [j,j + nNodes];
            
            s = [ones(1,3*nEl) -ones(1,6*nEl) ones(1,3*nEl) -ones(1,6*nEl)];
            
            %Create Laplacian summation matrix for calculating the gradient
            S = sparse(i, j, s);
            n = 1 - sum(S, 2);
            S = S + spdiags(n - 1, 0, 2*nNodes, 2*nNodes);
            
            %Create Hessian matrix
            H = S;
            
            isUnmoveable = this.IsConstrainedNode | this.IsFixedNode | this.IsBoundingBoxNode;
            nUnmoveable  = sum(isUnmoveable);
            isMoveable   = find(~isUnmoveable);
            isUnmoveable = find(isUnmoveable);

            H(isUnmoveable,:)          = 0;
            H(nNodes + isUnmoveable,:) = 0;
            H(:,isUnmoveable)          = 0;
            H(:,nNodes + isUnmoveable) = 0;
            
            H(isUnmoveable, isUnmoveable) = speye(nUnmoveable, nUnmoveable);
            H(nNodes + isUnmoveable, nNodes + isUnmoveable) = speye(nUnmoveable, nUnmoveable);

            %minimize energy functional
           	b                        = [x, y].';
            g                        = (S*b);
            g(isUnmoveable)          = 0;
            g(nNodes + isUnmoveable) = 0;

            dx            = (H \ g);
            x(isMoveable) = x(isMoveable) - 1 * dx(isMoveable).';
            y(isMoveable) = y(isMoveable) - 1 * dx(isMoveable + nNodes).';
            
            this.X = x;
            this.Y = y;
        end
        
        function this = updateBackgroundTriangulation(this)
            this.BackgroundTriangulation = DelaunayTri(this.X.', this.Y.', this.ConstrainedEdges.');
        end
                
        function this = removeVertices(this)
            %% #FIXME, this method does not remove vertices associated with constrained edges properly
            vertices  = unique(this.VertexRemovalQueue);
            vertices  = sort(vertices, 'descend');
            nVertices = numel(vertices);
            
            this.X(vertices)                 = [];
            this.Y(vertices)                 = [];
            this.IsConstrainedNode(vertices) = [];
            this.IsFixedNode(vertices)       = [];
            this.IsBoundingBoxNode(vertices) = [];
            
            ceEdges  = this.ConstrainedEdges;
            ceBound  = this.ConstrainedEdgeBoundaries;
            ceParam  = this.ConstrainedEdgeParameters;
            cePairs  = this.ConstrainedEdgePairs;
            elements = this.Elements;
            for i = 1:nVertices
                j            = (ceEdges == vertices(i));
                j            = any(j);
                ceEdges(:,j) = [];
                ceBound(:,j) = [];
                ceParam(:,j) = [];
                
                edges  = find(j);
                edges  = sort(edges,'descend');
                nEdges = numel(edges);
                for k = 1:nEdges
                    l = cePairs > edges(k);
                    cePairs(l) = cePairs(l) - 1;
                end
                
                j          = ceEdges > vertices(i);
                ceEdges(j) = ceEdges(j) - 1;
                
                k           = elements > vertices(i);
                elements(k) = elements(k) - 1;
            end
            
            this.ConstrainedEdges          = ceEdges;
            this.ConstrainedEdgeBoundaries = ceBound;
            this.ConstrainedEdgePairs      = cePairs;
            this.ConstrainedEdgeParameters = ceParam;
            this.Elements                  = elements;
            
            this.VertexRemovalQueue = [];
        end

        function this = doPostProcessing(this)
            %% remove elements not referenced by the mesh
            incenters   = this.ElementIncenters;
            
            notInDomain = ~inDomainHull(this,incenters(1,:),incenters(2,:));
            nElements   = sum(~notInDomain);
            
            this.Elements(:,notInDomain) = [];
            this.VertexRemovalQueue      = setdiff(1:numel(this.X), unique(this.Elements));
            this                         = this.removeVertices;
            this.BackgroundTriangulation = TriRep(this.Elements.', this.X.', this.Y.');
            
            this.ElementIncenters(:,notInDomain)     = [];
            this.ElementInradii(notInDomain)         = [];
            this.ElementCircumcenters(:,notInDomain) = [];
            this.ElementCircumradii(notInDomain)     = [];
            
            %% calculate element areas, reorder vertices in cc orienation
            x   = this.X(this.Elements);
            y   = this.Y(this.Elements);
            
            elA = 0.5 * ( x(1,:).*(y(2,:) - y(3,:)) + x(2,:).*(y(3,:) - y(1,:)) + x(3,:).*(y(1,:) - y(2,:)));
            
            isNegative                      = elA < 0;
            this.Elements([1,2],isNegative) = this.Elements([2,1],isNegative);
            this.ElementAreas               = abs(elA);
            
            %% get element regions
            incenters(:,notInDomain) = [];
            regions = this.Regions;
            nRegions = numel(regions);
            elementRegion = zeros(1,nElements);
            for iRegion = nRegions:-1:1
                I = regions(iRegion).Geometry.inOn(incenters(1,:), incenters(2,:));
                elementRegion(I) = iRegion;
            end
            
            this.ElementRegions = elementRegion;
        end
        
        function this = increaseElementOrder(this)
            if this.ElementOrder == 1
                %% get boundary elements and edges
                this.BoundaryEdges    = this.BackgroundTriangulation.freeBoundary.';
                this.BoundaryElements = this.BackgroundTriangulation.edgeAttachments(this.BoundaryEdges.');
                this.BoundaryElements = [this.BoundaryElements{:}];
            elseif this.ElementOrder == 2
                tic
                %% get edges and elements
                edges      = this.BackgroundTriangulation.edges;
                cEdges     = this.ConstrainedEdges;
                cParam     = this.ConstrainedEdgeParameters;
                cBound     = this.ConstrainedEdgeBoundaries;
                boundaries = this.Boundaries;
                nEdges     = length(edges);
                edge2El    = this.BackgroundTriangulation.edgeAttachments(edges(:,1),edges(:,2));
                elements   = this.Elements;
                nNodes     = length(this.X);
                nConst     = length(cEdges);
                
                %% add edge midpoints to the node list
                elements = [    elements(1,:);0*elements(1,:);
                                elements(2,:);0*elements(2,:);
                                elements(3,:);0*elements(3,:)];
                cEdges   = [cEdges zeros(2,nConst)];
                cParam   = [cParam zeros(2,nConst)];
                cBound   = [cBound zeros(1,nConst)];
                x        = [this.X (this.X(edges(:,1))+this.X(edges(:,2)))/2];
                y        = [this.Y (this.Y(edges(:,1))+this.Y(edges(:,2)))/2];
                
                %% add midpoint nodes to triangles
                %  adjust the position of constrained nodes
                for i = 1:nEdges
                    nNodes = nNodes + 1;
                    for j = 1:numel(edge2El{i})
                        k = edge2El{i}(j);
                        l = [2 0 4] * (  (elements([1 3 5],k) == edges(i,1))...
                                       + (elements([1 3 5],k) == edges(i,2)));

                        elements(l,k) = nNodes;
                    end
                    
                    c = find(  ((cEdges(1,:) == edges(i,1)) & (cEdges(2,:) == edges(i,2)))...
                              |((cEdges(1,:) == edges(i,2)) & (cEdges(2,:) == edges(i,1))),1);
                    
                    if ~isempty(c)
                        s  = (cParam(1,c)+cParam(2,c))/2;
                        xs = boundaries(cBound(c)).x(s);
                        ys = boundaries(cBound(c)).y(s);
                        
                        nConst               = nConst + 1;
                        cEdges(:,[c nConst]) = [cEdges(1,c) cEdges(2,c);nNodes nNodes];
                        cParam(:,[c nConst]) = [cParam(1,c) cParam(2,c);s      s     ];
                        cBound(nConst)       = cBound(c);

                        x(nNodes) = xs;
                        y(nNodes) = ys;
                    end
                end
                
                this.X        = x;
                this.Y        = y;
                this.Elements = elements;
               
                this.ConstrainedEdges          = cEdges;
                this.ConstrainedEdgeParameters = cParam;
                this.ConstrainedEdgeBoundaries = cBound;
               
                elements2 = [elements(1,:) elements(2,:) elements(4,:) elements(2,:);
                             elements(2,:) elements(3,:) elements(5,:) elements(4,:);
                             elements(6,:) elements(4,:) elements(6,:) elements(6,:)];
                        
                bt2 = TriRep(elements2.',this.X.',this.Y.');
               
                this.BoundaryEdges    = bt2.freeBoundary.';
                this.BoundaryElements = this.BackgroundTriangulation(1).edgeAttachments(this.BackgroundTriangulation(1).freeBoundary);
                this.BoundaryElements = [this.BoundaryElements{:}];
                
                this = detectEdgePairs(this);
            else
                error('MotorProto:MeshFactory','ElementOrder must be less than 3');
            end
        end
        
        function plot(this)
            fillArgs = cell(3*numel([this.Regions]), 1);
            
            nThis   = numel(this);
            iRegion = 1;
            for i = 1:nThis;
                regions  = this(i).Regions;
                nRegions = numel(regions);
                elements = this(i).Elements;
                triX     = this(i).X(elements);
                triY     = this(i).Y(elements);
                elRegion = this(i).ElementRegions;

                for j = 1:nRegions
                    elInRegion        	 = (elRegion == j);
                    iFillArg             = (iRegion-1) * 3;
                    fillArgs{iFillArg+1} = triX(:, elInRegion);
                    fillArgs{iFillArg+2} = triY(:, elInRegion);
                    fillArgs{iFillArg+3} = regions(j).Material.Color;
                    iRegion              = iRegion + 1;
                end
            end
            
            set(gcf, 'Renderer', 'OpenGL');
            set(gca, 'DrawMode', 'Fast');
            hold on;
           	fill(fillArgs{:});
            axis equal;
        end
    end
    
    methods(Static)
        function [I, X, Y] = findIntersectingEdges(xei, yei, xej, yej)
            leps = min(hypot(diff([xei, xej]), diff([yei, yej]))) * sqrt(eps);
            I    = cell(1, size(xei,2));
            X    = cell(1, size(xei,2));
            Y    = cell(1, size(xei,2));
            
            verti = (abs(max(xei) - min(xei)) < (abs(max(yei) - min(yei)) * sqrt(eps)));
           	horzi = (abs(max(yei) - min(yei)) < (abs(max(xei) - min(xei)) * sqrt(eps)));
                
            vertj = (abs(max(xej) - min(xej)) < (abs(max(yej) - min(yej)) * sqrt(eps)));
            horzj = (abs(max(yej) - min(yej)) < (abs(max(xej) - min(xej)) * sqrt(eps)));
            
            minxei = min(xei);
            maxxei = max(xei);
            minyei = min(yei);
            maxyei = max(yei);
            
            minxej = min(xej);
            maxxej = max(xej);
            minyej = min(yej);
            maxyej = max(yej);
            
            for i = 1:size(xei,2)
                den  = (xei(1,i)-xei(2,i))*(yej(1,:)-yej(2,:)) - (yei(1,i)-yei(2,i))*(xej(1,:)-xej(2,:));
                
                c1   = xei(1,i).*yei(2,i)-yei(1,i).*xei(2,i);
                c2   = xej(1,:).*yej(2,:)-yej(1,:).*xej(2,:);
                
                numx = c1*(xej(1,:) - xej(2,:)) - (xei(1,i) - xei(2,i))*c2;
                numy = c1*(yej(1,:) - yej(2,:)) - (yei(1,i) - yei(2,i))*c2;

                xint = numx ./ den;
                yint = numy ./ den;

                inxi  = (xint - leps > minxei(i)) & (xint + leps < maxxei(i));
                inyi  = (yint - leps > minyei(i)) & (yint + leps < maxyei(i));
                inxiv = (xint > minxei(i) - leps) & (xint < maxxei(i) + leps);
                inyih = (yint > minyei(i) - leps) & (yint < maxyei(i) + leps);

                inxj  = (xint - leps > minxej) & (xint + leps < maxxej);
                inyj  = (yint - leps > minyej) & (yint + leps < maxyej);
                inxjv = (xint > minxej - leps) & (xint < maxxej + leps);
                inyjh = (yint > minyej - leps) & (yint < maxyej + leps);
                
                J = ((inxj & inyj) | (inxj & horzj & inyjh) | (inyj & vertj & inxjv));
                if horzi(i)
                    J = J & (inxi & inyih);
                elseif verti(i)
                    J = J & (inyi & inxiv);
                else
                    J = J & (inxi & inyi);
                end
                
                X{i} = xint(J);
                Y{i} = yint(J);
                I{i} = find(J);
            end
        end
    end
    
    methods(Abstract)
        curves = getAuxillaryBoundaries(this);
        this   = setBoundaryMaxEdgeLength(this);
        this   = detectBoundaryPairs(this);
        this   = detectEdgePairs(this);
        this   = compileBoundaryData(this)
    end
    
   	methods (Sealed,Access = protected)
      function copyOut = copyElement(this)
         copyOut          = copyElement@matlab.mixin.Copyable(this);
         copyOut.Assembly = copy(this.Assembly);
      end
    end
end
