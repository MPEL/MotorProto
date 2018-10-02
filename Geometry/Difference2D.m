classdef Difference2D < Geometry2D
    %Difference2D.m Calculates the difference of two planes
    %   C = Difference2D(G1 ,G2) Creates a new plane C which is the portion
    %   of G1 which does not overlap with G2.
    %   
    %   When G1 and G2 are planes, the following statemets are equivalent
    %
    %       C = Difference2D(G1, G2)
    %       C = G1 - G2
    %       C = difference(G1, G2)
    %       C = G1.difference(G2);
    %
    %   Note that Difference2D(G1, G2) is not equivalent to Difference2D(G2, G1).
    %
    %   The internal edges of C are automatically discarded. If G1 and G2
    %   result in a disconnected planar region, the resulting connected
    %   domain boundaries can be recovered by examining the Domains
    %   property.
    %
    %   Example: Calculate the intersection of a rectangle and annullus.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 0.5, 'Width', 0.5, 'Base', 'Corner', 'Position',[0.5 0.5]);
    %
    %       G2 = Geometry2D.draw('Sector2D', 'Radius', [0.5, 1], 'Angle', pi/2);
    %
    %       G3 = G1 - G2;
    %       G4 = difference(G1, G2);
    %       G5 = G2 - G1;
    %       G6 = difference(G2, G1);
    %
    %       figure;subplot(3, 2, 1); axis equal;
    %       G1.plot;
    %       subplot(3, 2, 2); axis equal;
    %       G2.plot;
    %       subplot(3, 2, 3); axis equal;
    %       G3.plot;
    %       subplot(3, 2, 4); axis equal;
    %       G4.plot;
    %       subplot(3, 2, 5); axis equal;
    %       G5.plot;
    %       subplot(3, 2, 6); axis equal;
    %       G6.plot;
    %
    % Difference2D methods:
    %   Difference2D defines no new methods
    %
    % Difference2D properties:
    %   Difference2D defines no new properties
    %
    % Difference2D inheritance:
    %   Difference2D inherts methods and properties from Composite2D. See the 
    %   help for Composite2D for more information.
    %
    % See also Composite2D, Geometry2D, Geometry, Parameterizable, MotorProto
    
    properties
        PositiveGeometry
        NegativeGeometry
    end
    
    methods
        %% Constructor Method
        function this = Difference2D(pGeometry, nGeometry, varargin)
            if nargin~=0
                this.PositiveGeometry = pGeometry;
                this.NegativeGeometry = nGeometry;
                this.PlotStyle        = pGeometry.PlotStyle;
                nVarargin             = numel(varargin);
                for i = 1:2:nVarargin
                    this.(varargin{i}) = varargin{i+1};
                end
                this = build(this);
            end
        end
        
        %% Parameterization Update Methods
        function this = build(this)
         	curves = makeNonintersecting([this.PositiveGeometry.Curves, reverse(this.NegativeGeometry.Curves)]);
            [X, Y] = curves.getMidpoint;
            
            [inPos, onPos, normPos] = this.PositiveGeometry.inOn(X, Y);
            normPosX = normPos(:, 1);
            normPosY = normPos(:, 2);
            
            [inNeg, onNeg, normNeg] = this.NegativeGeometry.inOn(X, Y);
            normNegX = normNeg(:, 1);
            normNegY = normNeg(:, 2);
            
            innerProduct = normPosX.*normNegX + normPosY.*normNegY;
            
            midpointDistance = hypot(bsxfun(@minus, X, X.'), bsxfun(@minus, Y, Y.'));
            scaleFactor      = max(max(midpointDistance));
            areSameMidpoint  = midpointDistance < (scaleFactor * sqrt(eps));
                            
            duplicateInterfaces = any(tril(areSameMidpoint, -1), 1).';
            
            remove =  (inNeg) ...
                    | (onNeg & ~(inPos | onPos)) ...
                    | (onPos & onNeg & (innerProduct > sqrt(eps))) ...
                 	| (duplicateInterfaces);

            keep = ~remove;
            
            curves   = curves(keep);
            rotation = this.vRotation;
            if rotation ~= 0
                curves = rotate(curves, 'Rotation', rotation, 'Position', this.Position);
            end

            this.Curves = curves;
            this        = sortCurves(this);
        end
    end
end