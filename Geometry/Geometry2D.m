classdef Geometry2D < Geometry & matlab.mixin.Copyable
    %Geometry2D.m Is an abstract interface class for all planar geometry objects
    %
    % Geometry2D methods:
    %   plot         - Creates a plot of the plane
    %   wireframe    - Creates a wireframe plot of the plane
    %   area         - Calculates the area of the plane
    %   union        - Calculates the union of two planes
    %   intersection - Calculates the intersection of two planes
    %   difference   - Calculates the difference of two planes
    %   rotate       - Rotates the plane about a given axis
    %   sortCurves   - Determines the connected regions of the plane
    %   inOn         - Determine the In/On/Out status of a set of points
    %   draw         - Creates a new plane
    %
    % Geometry2D properties:
 	%   Dimension      - 2, The spatial dimension of a plane
    %   PlotStyle      - A cell array controlling the plot of the plane
    %   PlotResolution - The number of points to use when plotting the boundary
    %   Position       - The initial translation of the plane
    %   Rotation       - The initial rotation of the plane
    %   Base           - Controls how the position of the object is measured
    %   Curves         - The boundary curves of the plan
    %   Domains        - Arrays indicating the connected regions of the plane
    %
    % Geometry2D inheritance:
    %   Geometry2D inherts methods and properties from Geometry.
    %   See the help for Geometry for more information.
    %
    % See also Rect, Sector, Rotation2D, Polygon2D, Geometry, Parameterizable, 
    %          MaterialProperty, MotorProto
    
%{
properties:
 	%Dimension - 2, The spatial dimension of a plane
    %   This property indicates the object is 2-Dimensional (a plane). All 
    %   objects derived from the Geometry2D class have the same value for this
    %	property.
    %
    % See also Geometry2D
    Dimension;
    
    %PlotStyle - A cell array controlling the plot of the plane
    %   The PlotStyle property is a cell array containing arguments which are
    %   valid with MATLAB's patch function.
    %
    %   Example: Change the PlotStyle property of an arc
    %       G = Geometry2D.draw('Annulus2D', 'Radius', rand(1,2), 'Rotation', pi*rand(1), 'Angle', pi*rand(1), 'Position', rand(1,2));
    %       figure; subplot(1, 2, 1);
    %       G.plot; axis equal;
    %
    %       G.PlotStyle = {'y', 'FaceAlpha', 0.1};
    %       subplot(1, 2, 2);
    %       G.plot; axis equal;
    %   
    % See also patch, plot, wireframe, PlotResolution, Geometry2D
    PlotStyle;
    
    %PlotResolution - The number of points to use when plotting the boundary
    %   The PlotResolution property controls the accuracy of the representation
    %   of the curve when plotted.
    %
    %   Example: Change the PlotResolution to obtain a more accurate plot
    %       G = Geometry2D.draw('Annulus2D', 'Radius', [0.5, 1], 'Angle', pi, 'PlotResolution', 3);
    %       figure; subplot(1, 2, 1);
    %       G.plot; axis equal;
    %
    %       G.PlotResolution = 10;
    %       subplot(1, 2, 2);
    %       G.plot; axis equal;
    %       
    % See also patch, plot, wireframe, PlotStyle, Geometry2D
    PlotResolution;
    
    %Position - The initial translation of the plane
    %   The Position property indicates the initial amount the curve should be
    %   moved after it is created. This point is the default rotation axis of 
    %   the curve.
    %
    %   Example 1: Create two rectangles with different Position values.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1);
    %       G2 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1, 'Position',[-1, 0], 'PlotStyle', {'b'});
    %       figure;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Rectangle', 'Shifted Rectangle');
    %
    %   Example 2: The Position property is the axis about which the rotation
    %              angle specified by the Rotation property is applied.
    %       G1 = Geometry2D.draw('Annulus2D', 'Radius',[0.5, 1], 'Angle', pi/6);
    %
    %       G2 = Geometry2D.draw('Annulus2D', 'Radius',[0.5, 1], 'Angle', pi/6, 'Rotation', -pi/4, 'PlotStyle', {'b'}, 'Position',[1, 0]);   
    %       figure; axis equal;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Annulus', 'Rotated Annulus');
    %
    % See also Rotation, Geometry2D
    Position;
    
    %Rotation - The initial rotation of the plane
    %   The Rotation property indicates the amount object should be rotated
    %   about the location specified in the Position property.
    %
    %   Example 1: Create two lines with different Rotation values.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1);
    %       G2 = Geometry2D.draw('Rectangle2D','Length', 1, 'Width', 1, 'Rotation', pi/3, 'PlotStyle', {'b'});
    %       figure;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Rectangle', 'Rotated Rectangle');
    %
    %   Example 2: The Rotation property is applied after the object is
    %              translated from the origin to the location specified in the 
    %              Position property
    %       G1 = Geometry2D.draw('Annulus2D',...
    %                               'Radius',[0.5, 1],...
    %                               'Angle',pi/6);
    %
    %       G2 = Geometry2D.draw('Annulus2D', 'Radius', [0.5, 1], 'Angle', pi/6, 'Rotation', -pi/4, 'PlotStyle', {'b'}, 'Position', [1, 0]);  
    %       figure; axis equal;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Annulus', 'Rotated Annulus');
    %
    % See also Position, rotate, Geometry2D
    Rotation;
    
    %Base - Controls how the position of the object is measured
    %   The Base property controls how the position of the object is measured.
    %   The valid options are
    %   
    %       Center - The center of the plane
    %       Corner - The lower-left hand corner of the plane
    %
    %   Particular objects may only implement a subset of these options where 
    %   they make sense.
    %
    %   Example: Create two rectangles with different Base values.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1, 'Base', 'Center');
    %       G2 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1, 'Base',' Corner');
    %       figure; axis equal; hold on;
    %       G1.plot;
    %       G2.wireframe;
    %
    % See also Geometry2D
    Base;
    
    %Curves - The boundary curves of the plane
    %   The Curves property is a cell array containing curves describing the
    %   boundary region of the plane.
    %
    %   Example: Find the boundary curves for two disjoint regions
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1);
    %       G2 = Geometry2D.draw('Annulus2D', 'Radius', [0.1, 1], 'Angle', pi/2, 'Position', [-1.5, 0]);
    %       G3 = G1 + G2;
    %       figure; axis equal;
    %       G3.plot;
    %       G3.Curves{G3.Domains{1}}
    %       G3.Curves{G3.Domains{2}}
    %
    % See also Domains, Geometry2D
    Curves;
    
    %Domains - Arrays indicating the connected regions of the plane
    %   The Domains property is a cell array of integer vectors, indicating
    %   groups of boundary curves belonging to disjoint regions of the plane.
    %
    %   Example: The union of two non-overlapping planes results in an object
    %            having multiple domains.
    %       G1 = Geometry2D.draw('Rectangle2D', 'Length', 1, 'Width', 1);
    %       G2 = Geometry2D.draw('Annulus2D', 'Radius', [0.1, 1], 'Angle', pi/2, 'Position', [-2, 0]);
    %       G3 = G1 + G2;
    %       figure; axis equal;
    %       G3.plot;
    %       G3.Curves{G3.Domains{1}}
    %       G3.Curves{G3.Domains{2}}
    %
    % See also Curves, Geometry2D
    Domains;
%}
    
    properties (Constant=true)
        Dimension = 2;
        Plane = {'x', 'y', '0'};
    end
    
    properties         
        PlotStyle       = {'w'};
        PlotResolution  = 100;
        Base            = 'center'
    end
    
 	properties (SetAccess = protected)
        Curves
        Domains
    end
    
    properties (SetAccess = protected)
        Orientation = true;
        Negation    = false;
    end
    
    methods
        function this = Geometry2D
            this = this@Geometry;
        end
                
        function this = sortCurves(this)
            %sortCurves - Determines the connected regions of the plane
            % G = sortCurves(G) analyzes the boundary curves of G and arranges
            % them from start point to endpoint. This method also determines the
            % seperately connected regions of the plane indicated in the Domains
            % property.
            %
            % See also Geometry2D
            curveArray       = this.Curves;
            
            %% Remove ~Zero Length Curves
            dl          = curveArray.length;
            deps        = sqrt(eps) * max(dl);
            isZero      = (dl < deps);
            
            if any(isZero)
                warning('Zero length curves will be removed. Check geometry definition');
            end
            
            curveArray  = curveArray(~isZero);
            this.Curves = curveArray;
            
            nCurves = numel(curveArray);
            
            X0 = [curveArray.X0].';
            X1 = [curveArray.X1].';
            Y0 = [curveArray.Y0].';
            Y1 = [curveArray.Y1].';
            
            endpointDistances      = hypot(bsxfun(@minus,X0,X1.'), bsxfun(@minus,Y0,Y1.'));
            scaleFactor            = max(max(endpointDistances));
            [thisCurve, nextCurve] = find(endpointDistances < sqrt(eps)*scaleFactor);
            
            nextCurve   = nextCurve(thisCurve);
            isAssigned  = false(nCurves, 1);
            domainArray = cell(1, nCurves);
            k           = 0;
            while ~all(isAssigned)
                indUnassigned = find(~isAssigned);
                nUnassigned   = numel(indUnassigned);
                thisDomain    = zeros(1,nUnassigned);
                
                j = indUnassigned(1);
                i = 0;
                while i < nUnassigned && (j ~= thisDomain(1)) && ~isAssigned(j)
                    i             = i + 1;
                    thisDomain(i) = j;
                    isAssigned(j) = true;
                    j             = nextCurve(j);
                end
                k              = k + 1;
                domainArray{k} = thisDomain(1:i);
            end
            
            %% Assign outputs
            %this.Orientation = orientationArray;
            this.Domains     = domainArray(1:k);
        end

        function [In, On, N] = inOn(this, xIn, yIn)
        %inOn - Determine the In/On/Out status of a set of points
        %   [In,On,N] = inOn(G,X,Y) determines the position of a set of points
        %   relative to the plane. The vectors X and Y are a the x- and
        %   y-components, respectively, of a set of test points. The logical
        %   vector In is such that the points X(In) and Y(In) are interior to G.
        %   Similarly, the points X(On) and Y(On) are on G, with unit normals
        %   point to the interior of G given by N(On,:).
        %
        %   Example: Find the points which are interior to annulus and on an
        %            annulus. Plot the unit normals.
        %         G = Geometry2D.draw('Annulus2D', 'Radius', [0.5, 1], 'Angle', pi/2);
        %
        %         theta = linspace(-pi/6, 2*pi/3, 10).';
        %         X = [0.5*cos(theta); cos(theta); zeros(10,1); linspace(0.5,1,10).'; 1.1-1.2*rand(100,1)];
        %         Y = [0.5*sin(theta); sin(theta); linspace(0.5,1,10).'; zeros(10,1); 1.1-1.2*rand(100,1)];
        %         [In, On, N] = G.inOn(X, Y);
        %         figure; hold on; axis equal
        %         G.wireframe;
        %         scatter(X(In), Y(In), 'b', 'o');
        %         scatter(X(~In & ~On), Y(~In & ~On), 'r', 'x');
        %         quiver(X(On), Y(On), N(On,1), N(On,2));
        %
        % See also Geometry2D
        
            %% Preallocate
            nPoints = length(xIn);
            In      = false(nPoints, 1);
            On      = false(nPoints, 1);
            N       = zeros(nPoints, 2);
            nThis   = numel(this);
            
            for iThis = 1:nThis
                %% get Curves and preallocate
                curveArray	= this(iThis).Curves;
                nCurves     = numel(curveArray);
                wNumber     = zeros(nPoints, nCurves);
                onCurve     = false(nPoints, nCurves);
                normalX     = zeros(nPoints, nCurves);
                normalY     = zeros(nPoints, nCurves);

                %% test Points against each curve
                for iCurve = 1:nCurves           
                    [wNumber(:,iCurve), onCurve(:,iCurve), normalX(:,iCurve), normalY(:,iCurve)] = inOn(curveArray(iCurve), xIn, yIn);
                end
                wNumber = sum(wNumber, 2);
                OnI     = any(onCurve, 2);

                %% check validity of winding number
                isClosed = all(abs(imag(wNumber(~OnI))) < sqrt(eps));
                assert(isClosed, 'MotorProto:Geometry2D', 'Open regions not supported');
                wNumber = real(wNumber);

                isSimple = all(abs(wNumber(~OnI) - round(wNumber(~OnI))) < (sqrt(eps))) & ~ (any(wNumber(~OnI) < -0.5) & any(wNumber(~OnI) >  0.5));
                            
                assert(isSimple, 'MotorProto:Geometry2D', 'Self intersecting regions are not supported');
                wNumber = round(wNumber);
                
                %% assign outputs
                On = On | OnI;
                if this(iThis).Orientation
                    In = In | ((wNumber >= +1) & ~OnI);
                else
                    In = In | ((wNumber == +0) & ~OnI);
                end
                
             	%% calculate normals
                %if a point (xIn,yIn) is on more than one curve, calculate the mean
                %of all the normals at that point
                onCurve(~OnI,:) = false;
                [iPoints, ~]     = find(onCurve);
                if ~isempty(iPoints);
                    normalX = normalX(onCurve);
                    normalY = normalY(onCurve);

                    N(:,1) = N(:,1) + accumarray(iPoints, normalX, [nPoints, 1]);
                    N(:,2) = N(:,2) + accumarray(iPoints, normalY, [nPoints, 1]);
                end
            end

            normalNorm = sqrt(sum((N(On,:).^2), 2));
            N(On,1)    = N(On,1) ./ normalNorm;
            N(On,2)    = N(On,2) ./ normalNorm;

            %% check for errors
            assert(~any(any(isnan(N))), 'Geometry2D:inOn', 'Normal vectors returned NAN values');
        end
    end
    
    methods (Sealed)
        function gHandleOut = plot(this)
        	%plot - Creates a plot of the plane
            % plot(G) Creates a plot of the plane using the variables in the 
            % PlotStyle cell array.
            %
            %   Example: Create a Polygon and change the way it is plotted.
            %       pts = [0 0; 1 0.1; 0.9 1; -0.1 0.1];
            %       G   = Geometry2D.draw('Polygon2D', 'Points', pts, 'PlotResolution', 3);
            %       figure; subplot(1, 2, 1);
            %       G.plot;axis equal;
            %
          	%       G.PlotStyle      = {'y', 'FaceAlpha', 0.1};
            %       G.PlotResolution = 10;
         	%       subplot(1, 2, 2);
            %       G.plot; axis equal;
            %
            % See also Geometry2D
            
            curves      = [this.Curves];
            scaleFactor = max([curves.bbRadius]);
            nGeometry   = numel(this);
            for iGeometry = 1:nGeometry
                plotRes  = this(iGeometry).PlotResolution;
                nDomains = length(this(iGeometry).Domains);
                xPoints  = cell(nDomains, 1);
                yPoints  = cell(nDomains, 1);
                
                for iDomain = 1:nDomains
                    nDomainCurves       = length(this(iGeometry).Domains{iDomain});
                    nPoints             = zeros(1, nDomainCurves);
                    for iDomainCurve = 1:nDomainCurves
                        iCurve = this(iGeometry).Domains{iDomain}(iDomainCurve);
                        nPoints(iDomainCurve) = max(9, ceil(plotRes * this(iGeometry).Curves(iCurve).length / scaleFactor));
                        nPoints(iDomainCurve) = max(nPoints(iDomainCurve), 2);
                    end
                    
                    xPoints{iDomain,1}  = zeros(sum(nPoints), 1);
                    yPoints{iDomain,1}  = zeros(sum(nPoints), 1);
                    
                    for iDomainCurve = 1:nDomainCurves
                        iCurve      = this(iGeometry).Domains{iDomain}(iDomainCurve);
                        
                        iPointsLow  = sum(nPoints(1:iDomainCurve-1)) + 1;
                        iPointsHigh = sum(nPoints(1:(iDomainCurve)));
                        iPoints     = iPointsLow:iPointsHigh;
                        
                        sPoints     = linspace(0, 1, nPoints(iDomainCurve));
                        
                        xPoints{iDomain,1}(iPoints) = this(iGeometry).Curves(iCurve).x(sPoints);
                        yPoints{iDomain,1}(iPoints) = this(iGeometry).Curves(iCurve).y(sPoints);
                    end
                end

                if nDomains == 1
                    gHandleOut = patch(cell2mat(xPoints), cell2mat(yPoints), this(iGeometry).PlotStyle{:});
                elseif nDomains > 1
                    [f,v] = poly2fv(xPoints,yPoints);
                    gHandleOut = patch('Faces', f, 'Vertices', v, 'FaceColor', this(iGeometry).PlotStyle{1}, 'EdgeColor','None');

                   for iRegion = 1:numel(xPoints);
                       gHandleOut = line(xPoints{iRegion}, yPoints{iRegion}, 'Color', 'k');
                   end
                end
            end
            axis equal;
        end
        
        function gHandleOut = wireframe(this)
            %wireframe - Creates a wireframe plot of the plane
            % wireframe(G) Creates a wireframe plot of the plane. This method 
            % ignores the PlotStyle property but does monitor the PlotResolution
            % property.
            %
            %   Example: Create an Annulus and change the way it is plotted.
            %       G = Geometry2D.draw('Annulus2D', 'Radius', rand(1, 2),. 'Rotation', pi*rand(1), 'Angle',    pi), 'Position', rand(1, 2), 'PlotResolution', 3);
            %       figure; subplot(1, 2, 1);
            %       G.wireframe; axis equal;
            %
          	%       G.PlotStyle      = {'y', 'FaceAlpha', 0.1};
            %       G.PlotResolution = 10;
         	%       subplot(1, 2, 2);
            %       G.wireframe; axis equal;
            %
            % See also Geometry2D
            
            gHandleOut = plot([this.Curves]);
            axis equal;
            
            if numel(gHandleOut) == 1;
                gHandleOut = gHandleOut{1};
            end
        end

        function objOut = plus(obj1, obj2)
            obj1   = copy(obj1);
            obj2   = copy(obj2);
            objOut = Union2D([obj1, obj2]);
        end
        
        function objOut = minus(obj1, obj2)
            obj1   = copy(obj1);
            obj2   = copy(obj2);
            objOut = Union2D([reverse(obj1), obj2], 'Negation', true);
        end

        function objOut = mtimes(obj1, obj2)
            obj1   = copy(obj1);
            obj2   = copy(obj2);
            objOut = Union2D([reverse(obj1), reverse(obj2)], 'Negation', true);
        end
        
     	function objOut = union(varargin)
            %union - Calculates the union of two planes
            %   C = union(G1,G2) Creates a new plane C which is the union of the 
            %   two input planes G1 and G2. The internal edges are automatically 
            %   discarded. If G1 and G2 are disconnected, the boundary curves of
            %   each connected domain can be recovered by examining the domains
            %   property.
            %
            %   C = G1 + G2 is an equivalent syntax.
            %
            %   Example: Calculate the union of two overlapping polygons.
            %       pts1 = [0 0; 1 0.1; 0.9 1; -0.2 0.1];
            %       pts2 = [0 0; 0.1 0.9; -0.1 0; -0.1 -0.1; 0 -0.1];
            %       G1   = Geometry2D.draw('Polygon2D', 'Points', pts1);
            %       G2   = Geometry2D.draw('Polygon2D', 'Points', pts2);
            %       figure;subplot(1, 2, 1); axis equal; hold on;
            %       G1.wireframe;
            %       G2.wireframe;
            %
            %       G3 = G1 + G2;
            %       subplot(1, 2, 2); axis equal;
            %       G3.plot;
            %
            % See also Geometry2D, Composite2D, Union2D
            objIn  = copy([varargin{:}]);
          	objOut = Union2D(objIn);
        end
        
        function objOut = difference(obj1, obj2)
            %difference - Calculates the difference of two planes
            %   C = difference(G1,G2) Creates a new plane C which is the portion
            %   of G1 which does not overlap with G2.
            %
            %   C = G1 - G2 is an equivalent syntax.
            %
            %   The internal edges of C are automatically discarded. If G1 and 
            %   G2 result in a disconnected planar region, the resulting 
            %   connected domain boundaries can be recovered by examining the 
            %   Domains property.
            %
            %   C = G1 - G2 is an equivalent syntax.
            %
            %   Example: Calculate the difference of two overlapping polygons.
            %       pts1 = [0 0; 1 0.1; 0.9 1; -0.2 0.1];
            %       pts2 = [0 0; 0.1 0.9; -0.1 0; -0.1 -0.1; 0 -0.1];
            %       G1   = Geometry2D.draw('Polygon2D', 'Points', pts1);
            %       G2   = Geometry2D.draw('Polygon2D', 'Points', pts2);
            %       figure;subplot(1, 3, 1); axis equal; hold on;
            %       G1.wireframe;
            %       G2.wireframe;
            %
            %       G3 = G1 - G2;
            %       subplot(1, 3, 2); axis equal;
            %       G3.plot;
            %
            %       G4 = G2 - G1;
            %       subplot(1, 3, 3); axis equal;
            %       G4.plot;
            %
            % See also Geometry2D, Composite2D, Difference2D
            obj1   = copy(obj1);
            obj2   = copy(obj2);
            objOut = Union2D([reverse(obj1), obj2], 'Negation', true);
        end
        
        function objOut = intersection(varargin)
            %intersection - Calculates the intersection of two planes
            %   C = intersection(G1,G2) Creates a new plane C which is the
            %   overlapping portions of G1 and G2.
            %
            %   C = G1 * G2 is an equivalent syntax.
            %
            %   The internal edges of C are automatically discarded. If G1 and G2
            %   intersect on a boundary curve or a set of boundary curves, C is an
            %   open plane containing only those boundary curves. In this case,
            %   area(C) returns the area of the convex hull defined by the
            %   boundary curves.
            %
            %   Example: Calculate the intersection of two overlapping polygons.
            %       pts1 = [0 0; 1 0.1; 0.9 1; -0.2 0.1];
            %       pts2 = [0 0; 0.1 0.9; -0.1 0; -0.1 -0.1; 0 -0.1];
            %       G1   = Geometry2D.draw('Polygon2D', 'Points', pts1);
            %       G2   = Geometry2D.draw('Polygon2D', 'Points', pts2);
            %       figure;subplot(1, 2, 1); axis equal; hold on;
            %       G1.wireframe;
            %       G2.wireframe;
            %
            %       G3 = G1*G2;
            %       subplot(1, 2, 2); axis equal;
            %       G3.plot;
            %
            % See also Geometry2D, Composite2D, Intersection2D
            objIn  = copy([varargin{:}]);
            objOut = Intersection2D(reverse(objIn), 'Negation', true);
        end

        function this = rotate(this, rotation, position)
            nThis    = numel(this);
            if nThis > 0
                %% Rotate 2D-Geometry Axis
                [this.Axis, ~, ~] = rotate(this.Axis, rotation, position);
                
                %% Expand the data arrays to match the number of curves
                cDims = cellfun('length', {this.Curves});
                
            	if numel(rotation) > 1
                    rotation = arrayfun(@(x, y)(repmat(x, 1, y)), rotation, cDims, 'UniformOutput', false);
                    rotation = cell2mat(rotation);
                end

                if numel(position) > 2
                    position = [arrayfun(@(x, y)(repmat(x, y, 1)), position(:, 1), cDims.', 'UniformOutput', false), arrayfun(@(x, y)(repmat(x, y, 1)), position(:,2), cDims.', 'UniformOutput', false)];       
                    position = cell2mat(position);
                end
                
                %% Rotate Existing 1D-Geometry Axis
                rotate([this.Curves], rotation, position);
            end
        end
        
      	function varargout = reverse(varargin)
            for iArg = 1:nargin
                newOrientation               = ~[varargin{iArg}.Orientation];
                newOrientation               = num2cell(newOrientation);
                newNegation                  = ~[varargin{iArg}.Negation];
                newNegation                  = num2cell(newNegation);
                [varargin{iArg}.Orientation] = deal(newOrientation{:});
                [varargin{iArg}.Negation]    = deal(newNegation{:});
                [varargin{iArg}.Curves]      = reverse(varargin{iArg}.Curves(end:-1:1));
            end
            varargout = varargin;
        end
                
        function areaOut = area(this)
            %area - Calculates the area of the plane
            %   
            %   Example 1: Create a Polygon and calculate its area.
            %       pts = [0 0; 1 0.1; 0.9 1; -0.1 0.1];
            %       G   = Geometry2D.draw('Polygon2D', 'Points', pts);
            %       figure; axis equal;
            %       G.plot;
            %       G.area;
            %
            %   Example 2: Verify the area of the union of two polygons.
            %       pts1 = [0 0; 1 0.1; 0.9 1; -0.2 0.1];
            %       pts2 = [0 0; 0.1 0.9; -0.1 0; -0.1 -0.1; 0 -0.1];
            %       G1   = Geometry2D.draw('Polygon2D', 'Points', pts1);
            %       G2   = Geometry2D.draw('Polygon2D', 'Points', pts2);
            
            %       figure; axis equal; hold on;
            %       G1.wireframe;
            %       G2.wireframe;
            %       G3 = G1 + G2;
            %       G4 = G1*G2;
            %       G1.area + G2.area - G4.area
            %       G3.area
            %
            % See also Geometry2D
            
            nThis   = numel(this);
            areaOut = zeros(1, nThis);
            for i = 1:nThis
                curveArray  = this(i).Curves;
                nDomains    = length(this(i).Domains);
                nCurves     = numel(curveArray);
                curveArea 	= 0;
                xArray      = zeros(2*nCurves + 2*nDomains,1);
                yArray      = zeros(2*nCurves + 2*nDomains,1);
                iXyArray    = [-1; 0];
                for j = 1:nDomains
                    nDomainCurves = length(this(i).Domains{j});
                    for k = 1:nDomainCurves
                        iXyArray         = iXyArray + 2;
                        iCurve           = this(i).Domains{j}(k);
                        xArray(iXyArray) = curveArray(iCurve).x([0, 1]);
                        yArray(iXyArray) = curveArray(iCurve).y([0, 1]);
                        curveArea        = curveArea + curveArray(iCurve).area;
                    end
                    if nCurves > 0
                        %close the domain
                        iXyArray = iXyArray + 1;
                        iCurve   = this(i).Domains{j}(1);
                        xArray(iXyArray) = curveArray(iCurve).x(0);
                        yArray(iXyArray) = curveArray(iCurve).y(0);

                        %attach to the end of the first domain
                        iXyArray = iXyArray + 1;
                        iCurve   = this(i).Domains{1}(end);
                        xArray(iXyArray) = curveArray(iCurve).x(0);
                        yArray(iXyArray) = curveArray(iCurve).y(0);
                    end
                end
                areaOut(i) = curveArea + signedPolyArea(xArray, yArray);
            end
        end
    end
    
    methods (Static)
        function geometryOut = draw(typeIn, varargin)
            %draw - Creates a new plane
            % G = Geometry2D.draw(PlaneName,'PropertyName',propertyvalue,...)
            % creates a plane of type PlaneName and sets the properties given in
            % the PropertyName fields with the values in propertyvalue. The 
            % valid PlaneNames are
            %
            %   Rectangle2D - A rectangle defined by a height and width
            %   Annulus2D   - A circular annulus defined by two radii and a
            %                 subtended angle
            %   Polygon2D   - A polygon defined by a set of counterclockwise
            %                 oriented points.
            %
            %   Example 1 - Create a rectangle
            %       G = Geometry2D.draw('Rectangle2D', 'Width', rand, 'Length', rand 'Position', rand(1, 2), 'Rotation', pi*rand);
            %       figure; axis equal;
            %       G.plot;
            %
            %   Example 2 - Create an annulus
            %       r = rand;
            %       G = Geometry2D.draw('Annulus2D', 'Radius', [r/2, r], 'Angle', pi*rand, 'Position', rand(1, 2), 'Rotation', pi*rand);
            %       figure; axis equal;
            %       G.plot;
            %
            %   Example 3 - Create a polygon
            %       pts = [rand rand; -rand rand; -rand -rand; rand -rand];
            %       G = Geometry2D.draw('Polygon2D', 'Points', pts, 'Position', rand(1, 2), 'Rotation', pi*rand);
            %       figure; axis equal;
            %       G.plot;
            %
            % See also Geometry2D, Parameterizable, Rect, Sector, Polygon2D
            
            switch lower(typeIn)
                case {'rect', 'rect2d', 'rectangle', 'rectangle2d'}
                    geometryOut = Rect(varargin{:});
                case {'sector', 'sector2d', 'annulus', 'annulus2d'}
                    geometryOut = Sector(varargin{:});
                case 'curvewise'
                    geometryOut = Curvewise(varargin{:});
                case {'polygon', 'polygon2d'}
                    geometryOut = Polygon2D(varargin{:});
                otherwise
                    error('Geometry2D:draw', 'Unable to draw %s', typeIn);
            end
        end
    end
    
    methods (Abstract)
        this = build(this)
    end
    
    methods (Access = protected)
        function copyObj = copyElement(this)
            copyObj        = copyElement@matlab.mixin.Copyable(this);
            copyObj.Curves = copy(copyObj.Curves);
        end
    end
end