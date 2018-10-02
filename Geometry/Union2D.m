classdef Union2D < Geometry2D
    %Union2D.m Calculates the union of two planes
    %   C = Union2D(G1,G2) Creates a new plane C which is the union of the
    %   two input planes G1 and G2. 
    % 
    %   When G1 and G2 are planes, the following are equivalent statements
    %
    %       C = Union2D(G1,G2);
    %       C = G1 + G2
    %       C = union(G1,G2);
    %       C = G1.union(G2);
    %
    %   The internal edges of C are automatically discarded. If G1 and G2 are 
    %   disconnected, the boundary curves of each connected domain can be 
    %   recovered by examining the domains property.
    %
    %   Example: Calculate the union of a rectangle and annullus.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 0.5, 'Width', 0.5, 'Base', 'Corner', 'Position', [0.5, 0.5]);
    %
    %       G2 = Geometry2D.draw('Sector2D', 'Radius', [0.5, 1], 'Angle', pi/2);
    %
    %       G3 = G1 + G2;
    %       G4 = union(G1,G 2);
    %
    %       figure;subplot(2, 2, 1); axis equal;
    %       G1.plot;
    %       subplot(2, 2, 2); axis equal;
    %       G2.plot;
    %       subplot(2, 2, 3); axis equal;
    %       G3.plot;
    %       subplot(2, 2, 4); axis equal;
    %       G4.plot;
    %
    % Union2D methods:
    %   Union2D defines no new methods
    %
    % Union2D properties:
    %   Union2D defines no new properties
    %
    % Union2D inheritance:
    %   Union2D inherts methods and properties from Composite2D. See the help
    %   for Composite2D for more information.
    %
    % See also Composite2D, Geometry2D, Geometry, Parameterizable, MotorProto
    
    properties (SetAccess = protected)
        InputGeometry
    end
    
    methods
        %% Constructor Method
        function this = Union2D(geometry, varargin)
            this = this@Geometry2D;
            if nargin~=0
                this.InputGeometry = geometry;
                this.PlotStyle = geometry(1).PlotStyle;
                for i = 1:2:(nargin - 1)
                    this.(varargin{i}) = varargin{i+1};
                end
                this = build(this);
            end
        end
        
        function this = build(this)
            curves = makeNonintersecting(copy([this.InputGeometry.Curves]));
            [X, Y] = curves.getMidpoint;
            
         	nPlanes = numel(this.InputGeometry);
            nCurves = numel(curves);
            
            inPlane    = false(nCurves, nPlanes);
            onBoundary = false(nCurves, nPlanes);
            normalX    = zeros(nCurves, nPlanes);
            normalY    = zeros(nCurves, nPlanes);
            
            for i = 1:nPlanes
                [inPlane(:,i), onBoundary(:,i), normal] = this.InputGeometry(i).inOn(X, Y);
                normalX(:,i) = normal(:,1);
                normalY(:,i) = normal(:,2);
            end
            
            inPlane      = any(inPlane, 2);
            innerProduct =  bsxfun(@times, normalX, reshape(normalX, nCurves, 1, nPlanes))...
                          + bsxfun(@times, normalY, reshape(normalY, nCurves, 1, nPlanes)); %TODO, FIXME - Out of memory error for large models
            innerProduct = min(min(innerProduct, [], 3), [], 2);
            
            internalInterface = any(onBoundary, 2) & (innerProduct < -sqrt(eps));
            
            midpointDistance = hypot(bsxfun(@minus, X, X.'), bsxfun(@minus, Y, Y.'));
            areSameMidpoint  = midpointDistance < max(max(midpointDistance))*sqrt(eps);
                            
            duplicateInterfaces = any(tril(areSameMidpoint, -1), 1).';
            
            remove =  inPlane | duplicateInterfaces | internalInterface;
            keep   = ~remove;
            
            curves = curves(keep);
            
            if any(keep)
                rotation = this.Rotation;
                if rotation ~= 0
                    curves = rotate(curves, 'Rotation', rotation, 'Position', this.Position);
                end

                if this.Negation
                    curves = reverse(curves);
                end
            end
            
            this.Curves = curves;
            this        = sortCurves(this);
        end
    end
end