classdef Geometry1D < Geometry & matlab.mixin.Copyable
    %Geometry1D.m An abstract interface class for all 1-D geometry objects
    %
    % Geometry1D methods:
    %   plot         - Creates a line plot of the curve
    %   wireframe    - Creates a line plot of the curve
    %   rotate       - Rotates the curve about a particular axis
    %   intersection - Calculates the intersection points of two curves
    %   split        - Splits the curve into two or more peices
    %   reverse      - Reverses the orientation of the curve
    %   draw         - Creates a new curve
    %   x            - Calculates x values on the curve from a parameter vector
    %   y            - Calculates y values on the curve from a parameter vector
    %   dx           - Calculates dx/ds on the curve from a parameter vector
    %   dy           - Calculates dy/ds on the curve from a parameter vector
    %   length       - Calculates the length of the curve
    %   coincidenceType - Returns a flag indicating if the two curves overlap
    %   inOn         - Determine the In/On/Out status of a set of points
    %   cart2s       - Convert a set of points to parameter values
    %
    % Geometry1D properties:
    %   Dimension      - 1, The spatial dimension of a curve
    %   PlotStyle      - A cell array controlling the plot of the curves
    %   PlotResolution - The number of points to use when plotting the curve
    %   Orientation    - The orientation of the curve
    %   Position       - The initial translation of the curve
    %   Rotation       - The initial rotation of the curve
    %
    % Geometry1D inheritance:
    %   Geometry0D inherts methods and properties from Geometry. See the help
    %   for Geometry for more details.
    %
    % See also Arc, Polyline, Rotation1D, Geometry, Parameterizable, MotorProto
    
%{
properties:    
    %Dimension - 1, The spatial dimension of a curve
    %   This property indicates the object is 1-Dimensional (a curve). All 
    %   objects derived from the Geometry1D class have the same value for this
    %	property.
    %
    % See also Geometry1D
    Dimension;
    
    %PlotStyle - A cell array controling the plot of the curve
    %   The PlotStyle property is a cell array containing arguments which are
    %   valid with MATLAB's line function.
    %
    %   Example: Change the PlotStyle property of an arc
    %       G = Geometry1D.draw('Arc1D', 'Radius',   rand(1), 'Rotation', pi*rand(1), 'Angle',    pi*rand(1), 'Position', rand(1,2));
    %       figure; subplot(1, 2, 1);
    %       G.plot; axis equal;
    %
    %       G.PlotStyle = {'Color', 'r', 'LineStyle', ':'};
    %       subplot(1, 2, 2);
    %       G.plot; axis equal;
    %   
    % See also line, plot, wireframe, PlotResolution, Geometry1D
    PlotStyle;
    
    %PlotResolution - The number of points to use when plotting the curve
    %   The PlotResolution property controls the accuracy of the representation
    %   of the curve when plotted.
    %
    %   Example: Change the PlotResolution to obtain a more accurate plot;
    %       G = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi, 'PlotResolution', 3, 'PlotStyle', {'Color', 'k', 'Marker', 'o'});
    %       figure;subplot(1, 2, 1);
    %       G.plot; axis equal;
    %
    %       G.PlotResolution = 10;
    %       subplot(1, 2, 2);
    %       G.plot; axis equal;
    %       
    % See also line, plot, wireframe, PlotStyle, Geometry1D
    PlotResolution;
    
    %Orientation - The orientation of the curve
    %   The Orientation property indicates whether the curve is oriented
    %   counterclockwise or clockwise.
    %   
    %   G.Orientaiton = true sets the curve to counter-clockwise orientation and
    %   is the default.
    %
    %   G.Orientation = false sets the curve to clockwise orientation.
    %
    %   Curves are defined in a right handed manner. The 'interior' of a curve
    %   is defined using this right handedness.
    %
    %   Example 1: Visualize the 'interior' of an arc.
    %       G = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi/3);
    %       X = 1.25 - 2.5*rand(100, 1);
    %       Y = 1.25 - 2.5*rand(100, 1);
    %       figure;
    %       for i=1:2
    %           subplot(1, 2, i);
    %           In = G.inOn(X, Y);
    %           G.plot; axis equal; hold on;
    %           scatter(X(In), Y(In), 'o', 'b');
    %           scatter(X(~In), Y(~In), 'x', 'r');
    %           legend('Arc', 'Interior', 'Exterior')
    %           G.Orientation = false;
    %       end
    %
    %   Example 2: Visualize the 'interior' of a line.
    %       G = Geometry1D.draw('Line1D', 'Points', [0 0;1 1]);
    %       X = 1-2*rand(100, 1);
    %       Y = 1-2*rand(100, 1);
    %       figure;
    %       for i = 1:2
    %           subplot(1, 2, i);
    %           In = G.inOn(X, Y);
    %           G.plot; axis equal; hold on;
    %           scatter(X(In), Y(In), 'o', 'b');
    %           scatter(X(~In), Y(~In), 'x', 'r');
    %           legend('Line', 'Interior', 'Exterior')
    %           G.Orientation = false;
    %       end
    %
    % See also reverse, inOn, Geometry1D
    Orientation;
    
   	%Position - The initial translation of the object
    %   The Position property indicates the initial amount the curve should be
    %   moved after it is created. This point is the default rotation axis of 
    %   the curve.
    %
    %   Example 1: Create two lines with different Position values.
    %       G1 = Geometry1D.draw('Line1D', 'Points', [0.5 0.5;1 1]);
    %       G2 = Geometry1D.draw('Line1D', 'Points', [0.5 0.5;1 1], 'Position', [-1 0], 'PlotStyle', {'Color', 'b'});
    %       figure;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Line', 'Shifted Line');
    %
    %   Example 2: The Position property is the axis about which the rotation
    %              angle specified by the Rotation property is applied.
    %       G1 = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi/6, 'Position', [1 0]);
    %
    %       G2 = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi/6, 'Rotation', -pi/4, 'PlotStyle', {'Color', 'b'}, 'Position', [1 0]);   
    %       figure; axis equal;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Line', 'Rotated Line');
    %
    % See also Rotation, Geometry1D
    Position;
    
   	%Rotation - The initial rotation of the object
    %   The Rotation property indicates the amount object should be rotated
    %   about the location specified in the Position property.
    %
    %   Example 1: Create two lines with different Rotation values.
    %       G1 = Geometry1D.draw('Line1D', 'Points', [0.5 0.5;1 1]);
    %       G2 = Geometry1D.draw('Line1D', 'Points', [0.5 0.5;1 1], 'Rotation', pi/3, 'PlotStyle', {'Color', 'b'});
    %       figure;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Line', 'Rotated Line');
    %
    %   Example 2: The Rotation property is applied after the object is
    %              translated from the origin to the location specified in the 
    %              Position property
    %       G1 = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi/6, 'Position', [1 0]);
    %
    %       G2 = Geometry1D.draw('Arc1D', 'Radius',1, 'Angle', pi/6, 'Rotation', -pi/4, 'PlotStyle', {'Color', 'b'}, 'Position', [1 0]);   
    %       figure; axis equal;
    %       G1.plot; hold on;
    %       G2.plot;
    %       legend('Original Line', 'Rotated Line');
    %
    % See also Position, rotate, Geometry1D
    Rotation;
%}

    properties (Constant)
        Dimension = 1;
    end
    
    properties
        PlotStyle      = {'Color', 'k'};
        PlotResolution = 100;
        Orientation    = true;
        MaxEdgeLength  = inf;
    end
    
	properties (Abstract, Dependent)
        Order
    end
    
    methods (Abstract, Access = protected)
        this = updateCache(this);
    end
    
    properties (SetAccess = private, Dependent)
        IsClosed
    end
    
    properties (SetAccess = protected)
        X0 = 0;
        Xm = 0;
        X1 = 0;
        
        Y0 = 0;
        Ym = 0;
        Y1 = 0;
    end
    
    methods
        function this = Geometry1D(varargin)
            this = this@Geometry(varargin{:});
        end
        
        function this = set.Orientation(this, orientationIn)
            if ~(this.Orientation == orientationIn)
                this.Orientation = orientationIn;
                this = updateCache(this);
            end
        end
        
        function bool = get.IsClosed(this)
            x0   = [this.X0];
         	x1   = [this.X1];
         	y0   = [this.Y0];
         	y1   = [this.Y1];
           	l    = this.length;
            d    = hypot(x0 - x1, y0 - y1);
            bool = (d < l * sqrt(eps));
        end
        
        function [s, ds, n, objs] = split(s)
            %split - Splits the curve into two or more peices
            %   C = split(G,S) returns a cell array of curves C which are
            %   created by splitting the curve G and the parameter locations
            %   specified in S.
            %
            %   Example: Draw a line, split it at two points, and change the
            %            PlotStyle of one segment.
            %       G1 = Geometry1D.draw('Line1D','Points',rand(2,2));
            %       S  = [0.25;0.75];
            %       Gn = G1.split(S);
            %       figure;subplot(1,2,1);axis equal;
            %       G1.plot;
            %       Gn{2}.PlotStyle = {'Color','b','LineStyle',':'};
            %       subplot(1,2,2);hold on;axis equal;
            %       for i = 1:3
            %           Gn{i}.plot;
            %       end
            %
            % See also Geometry1D
            
            %% object specific behavior is implemented in subclasses
            s(s==0)	= [];
            s(s==1)	= [];
            s     	= sort([0; s; 1]);
            ds   	= diff(s);
            n      	= length(ds);
            objs   	= cell(n, 1);
        end
    end
    
  	methods (Sealed)
        function gHandleOut = plot(this)
            %plot - Creates a line plot of the curve
            %   plot(G) Creates a line plot of the curve using the style 
            %   specified in the PlotStyle property with the number of points 
            %   specified in PlotResolution.
            %   
            %   Example: Plot a line.
            %       G1 = Geometry1D.draw('Line1D', 'Points', [0 0;1 1]);
            %       figure; axis equal;
            %       G1.plot
            %
            % See also Geometry1D
            
            nThis      = numel(this);
            gHandleOut = cell(nThis, 1);
            hold on;
            for iCurve = 1:nThis
                sArray             = linspace(0, 1, this(iCurve).PlotResolution);
                xPlot              = this(iCurve).x(sArray);
                yPlot              = this(iCurve).y(sArray);
                plotOptions        = this(iCurve).PlotStyle;
                gHandleOut{iCurve} = line(xPlot, yPlot, plotOptions{:});
            end
            axis equal;
            
            if nThis == 1
                gHandleOut = gHandleOut{1};
            end
        end
        
        function arrow(this)
            plot(this);
            nCurves = numel(this);
            x       = zeros(nCurves, 1);
            y       = zeros(nCurves, 1);
            dx      = zeros(nCurves, 1);
            dy      = zeros(nCurves, 1);
            for iCurve = 1:nCurves
                x(iCurve)  = this(iCurve).x(0.5);
                y(iCurve)  = this(iCurve).y(0.5);
                dx(iCurve) = this(iCurve).dx(0.5);
                dy(iCurve) = this(iCurve).dy(0.5);
            end
            quiver(x, y, dx, dy);
        end
        
        function wireframe(this)
            %wireframe - Creates a line plot of the curve
            %   wireframe(G) Creates a line plot of the curve using the style 
            %   specified in the PlotStyle property with the number of points 
            %   specified in PlotResolution.
            %   
            %   Example: Plot a line.
            %       G1 = Geometry1D.draw('Line1D', 'Points', [0 0;1 1]);
            %       figure; axis equal;
            %       G1.wireframe;
            %
            % See also Geometry1D
            
            sArray      = linspace(0, 1, this.PlotResolution);
            xPlot       = this.x(sArray);
            yPlot       = this.y(sArray);
            plotOptions = this.PlotStyle;
            line(xPlot, yPlot, plotOptions{:});
            axis equal;
        end
        
      	function varargout = reverse(varargin)
            %reverse - Reverses the orientation of the curve
            %   C = reverse(G) returns a curve C identical to the curve G except
            %   with its orientation reversed.
            %
            %   Example: Reverse the orientation of a arc. Plot a few vectors
            %            tangent to the curve.
            %       G1  = Geometry1D.draw('Arc1D', 'Radius',  rand, 'Rotation', 4*pi*rand - 2*pi, 'Position', rand(1,2), 'Angle', 4*pi*rand - 2*pi);
            %       S   = 0:0.25:1;
            %       X1  = G1.x(S);
            %       Y1  = G1.y(S);
            %       dX1 = G1.dx(S);
            %       dY1 = G1.dy(S);
            %
            %       figure; subplot(1,2,1); hold on; axis equal;
            %       G1.plot;
            %       quiver(X1,Y1,dX1,dY1);
            %
            %       G2  = G1.reverse;
            %       X2  = G2.x(S);
            %       Y2  = G2.y(S);
            %       dX2 = G2.dx(S);
            %       dY2 = G2.dy(S);
            %
            %       subplot(1,2,2); hold on; axis equal;
            %       G2.plot;
            %       quiver(X2,Y2,dX2,dY2);
            %
            % See also Geometry1D
            for iArg = 1:nargin
                newOrientation               = ~[varargin{iArg}.Orientation];
                newOrientation               = num2cell(newOrientation);
                [varargin{iArg}.Orientation] = deal(newOrientation{:});
            end
            varargout = varargin;
        end
        
      	function newCurves = makeNonintersecting(this)
         	[~, ~, sParams] = intersection(this);
            
            classes       = this.getElementClasses;
            uniqueClasses = unique(classes);
            nUnique       = numel(uniqueClasses);
            newCurves     = cell(nUnique, 1);
            for iUnique = 1:nUnique
                classIndex         = strcmp(uniqueClasses{iUnique}, classes);
                newCurves{iUnique} = split(this(classIndex), sParams(classIndex));
            end
            newCurves = [newCurves{:}];
        end
        
        function newCurves = makeUnique(this)
            newCurves = makeNonintersecting(this);
            
            X = [newCurves.Xm];
            Y = [newCurves.Ym];
            
            dR     = hypot(bsxfun(@minus, X, X.'), bsxfun(@minus, Y, Y.'));
            scale  = max(max(dR));
            isSame = any(tril(dR < scale * sqrt(eps),-1));
            
            newCurves(isSame) = [];
        end
        
        function [X,Y] = getMidpoint(this)
            X = [this.Xm].';
            Y = [this.Ym].';
        end

     	function [X,Y,S] = intersection(this)
            %intersection - Calculates the intersection points of two curves
            %   [X,Y,S] = intersection(G1,G2) calculates the X and Y
            %   coordinates of the intersection points of the curves G1 and G2.
            %   This method also returns a 2 by n parameter matrix S such that
            %   corresponding the parameters of the intersection points.
            %
            %   Example: Calculate the intersection of an arc and a line. Plot
            %            the normal vectors at the points of intersection.
            %       G1 = Geometry1D.draw('Line1D','Points',[0 1;1 0]);
            %      	G2 = Geometry1D.draw('Arc1D',...
            %                            'Radius',  0.75,...
            %                            'Position',[0.1 0.1],...
            %                            'Angle',   pi/2);
            %       [X,Y,S] = intersection(G1,G2);
            %       dX1     = -G1.dy(S(:,1));
            %       dY1     =  G1.dx(S(:,1));
            %       dX2     = -G2.dy(S(:,2));
            %       dY2     =  G2.dx(S(:,2));
            %       figure;hold on;axis equal;
            %       G1.plot;
            %       G2.plot;
            %       quiver(X,Y,dX1,dY1);
            %       quiver(X,Y,dX2,dY2);
            %
            % See also Geometry1D
            
         	[I ,J]         = boundingBallsOverlap(this);
            classes        = getElementClasses(this);
            uniqueClasses  = unique(classes);
            nUniqueClasses = numel(uniqueClasses); 
            nThis          = numel(this);
            
            globalToLocal  = zeros(nThis, 1);
            isClass        = cell(nUniqueClasses, 1);
            for k = 1:nUniqueClasses
                isClassK                   = strcmp(uniqueClasses(k), classes);
                isClass{k}                 = isClassK;
                nkClass                    = sum(isClassK);
                globalToLocal(isClassK, 1) = (1:nkClass).';
            end
            
            X      = cell(nUniqueClasses^2, 1);
            Y      = cell(nUniqueClasses^2, 1);
            S      = cell(1, 2*nUniqueClasses^2);
            [S{:}] = deal(cell(nThis, 1));
            for i = 1:nUniqueClasses
                for j = 1:nUniqueClasses
                    %get classes logical array
                    isClassI  = isClass{i};
                    isClassJ  = isClass{j};
                    
                    %find where the I and J class bounding balls overlap
                    compareI  = isClassI(I);
                    compareJ  = isClassJ(J);
                    compareIJ = compareI & compareJ;
                    
                    %get the bounding ball overlap indices
                    globalI   = I(compareIJ);
                    globalJ   = J(compareIJ);
                    
                    %convert these to index into the "this" array
                    localI    = globalToLocal(globalI);
                    localJ    = globalToLocal(globalJ);
                    
                    %find the intersection points
                    k         = nUniqueClasses*(i-1) + j;
                    [X{k}, Y{k}, S{2*k-1}(isClassI), S{2*k}(isClassJ)] = generalIntersection(this(isClassI), this(isClassJ), localI, localJ);
                end
            end
            
            X = cell2mat(X);
            Y = cell2mat(Y);
            S = cellfun(@(varargin)(horzcat(varargin{:})), S{:}, 'UniformOutput', false);

            for kThis = 1:nThis
                s        = sort(S{kThis});
                ds       = [1, diff(s)];
                ds       = abs(ds);
                willKeep = (ds > sqrt(eps)) & (s > 0) & (s < 1);
                S{kThis} = s(willKeep);
            end
        end
        
        function this = rotate(this, rotation, position)
            nThis = numel(this);
            if nThis > 0
                [this.Axis, ~, ~] = rotate([this.Axis], rotation, position);
                for iThis = 1:nThis
                    updateCache(this(iThis));
                end
            end
        end
        
        function L = length(this)
            L = zeros(size(this));
            nThis = numel(this);
            for iThis = 1:nThis
                L(iThis) = elementLength(this(iThis));
            end
        end
        
        function N = MinEdgeNumber(this)
            N = zeros(size(this));
            nThis = numel(this);
            for iThis = 1:nThis
                N(iThis) = elementMinEdgeNumber(this(iThis));
                N(iThis) = max(N(iThis), ceil(this(iThis).length / this(iThis).MaxEdgeLength));
            end
        end
    end
    
    methods (Static)
        function geometryOut = draw(typeIn,varargin)        
            %draw - Creates a new curve
            % G = Geometry1D.draw(CurveName,'PropertyName',propertyvalue,...)
            % creates a curve of type CurveName and sets the properties given in
            % the PropertyName fields with the values in propertyvalue. The 
            % valid CurveNames are
            %
            %   Line1D - A straight line defined by two endpoints
            %   Arc1D  - A circular arc defined by a radius and subtended angle
            %
            % Example 1: Create a line.
            %   G = Geometry1D.draw('Line1D', 'Points', rand(2,2));
            %   figure; axis equal;
            %   G.plot;
            %
            % Example 2: Create an arc.
            %   G = Geometry1D.draw('Arc1D', 'Radius', rand, 'Angle', 2*pi*rand);
            %   figure; axis equal;
            %   G.plot;
            %
            % See also Geometry1D, Parameterizable, Polyline, Arc
            
            switch typeIn
                case {'Arc', 'arc', 'Arc1D', 'arc1D', 'Arc1d', 'arc1d'}
                    geometryOut = Arc(varargin{:});
                case {'Polyline', 'polyline', 'Line', 'line', 'Line1D', 'line1D', 'Line1d', 'line1d'}
                    geometryOut = Polyline(varargin{:});
                otherwise
                    error('Geometry1D:draw', 'Unknown Geometry1D subclass %s', typeIn);
            end
        end
    end
    
    methods (Abstract)
    	%x - Calculates x values on the curve from a parameter vector
        %   X = x(G,S) returns a vector of y-coordinates of points lying on the
        %   curve G which correspond to the parameters in the column vector S.
        %   Curves are only defined to exist on 0 <= S <= 1 but this method will
        %   return values of outside of the unit interval.
        %
        %   Example: Find the midpoint of a line
        %       G = Geometry1D.draw('Line1D', 'Points', rand(2,2));
        %       X = G.x(0.5);
        %       Y = G.y(0.5);
        %       figure; hold on; axis equal;
        %       G.plot;
        %       scatter(X, Y);
        %       legend('Line', 'Midpoint');
        %
        % See also y, dx
        xOut = x(this, s)
        
        %y - Calculates y values on the curve from a parameter vector
        %   Y = y(G,S) returns a vector of y-coordinates of points lying on the
        %   curve G which correspond to the parameters in the column vector S.
        %   Curves are only defined to exist on 0 <= S <= 1 but this method will
        %   return values of outside of the unit interval.
        %
        %   Example: Find the midpoint of a line
        %       G = Geometry1D.draw('Line1D', 'Points', rand(2,2));
        %       X = G.x(0.5);
        %       Y = G.y(0.5);
        %       figure; hold on; axis equal;
        %       G.plot;
        %       scatter(X, Y);
        %       legend('Line','Midpoint');
        %
        % See also x, dy
        yOut = y(this, s)
        
        %dx - Calculates dx/ds on the curve from a parameter vector
        %   dXdS = dx(G,S) returns a vector dXdS of values corresponding to the
        %   derivative of the x-component of the curve with respect to the
        %   parameter, at the parameter values specified in S. Curves are only 
        %   defined to n 0 <= S <= 1 but this method return values outside of
        %   the unit interval.
        %
        %   Example: Plot vectors tangent to an arc
        %       G    = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi);
        %       S    = (0:0.1:1).';
        %       X    = G.x(S);
        %       Y    = G.y(S);
        %       dXdS = G.dx(S);
        %       dYdS = G.dy(S);
        %       figure; axis equal; hold on;
        %       G.plot;
        %       quiver(X, Y, dXdS, dYdS);
        %
        % See also dx, y
        dxOut = dx(this, s)
        
       	%dy - Calculates dy/ds on the curve from a parameter vector
        %   dYdS = dx(G,S) returns a vector dYdS of values corresponding to the
        %   derivative of the y-component of the curve with respect to the
        %   parameter, at the parameter values specified in S. Curves are only 
        %   defined to n 0 <= S <= 1 but this method return values outside of
        %   the unit interval.
        %
        %   Example: Plot vectors tangent to an arc
        %       G    = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi);
        %       S    = (0:0.1:1).';
        %       X    = G.x(S);
        %       Y    = G.y(S);
        %       dXdS = G.dx(S);
        %       dYdS = G.dy(S);
        %       figure; axis equal; hold on;
        %       G.plot;
        %       quiver(X, Y, dXdS, dYdS);
        %
        % See also dx, y
        dyOut = dy(this, s)
        
        %length - Calculates the length of the curve
        lengthOut = elementLength(this)
        
        %dx2 - Calculates d^2x/ds^2 of the curve from a parameter vector
        dx2Out = dx2(this, s)
        
        %dy2 - Calculates d^2y/ds^2 of the curve from a parameter vector
        dy2Out = dy2(this, s)
        
        %area - The area under the curve using its endpoints as the x-axis
        %   a = area(G) computes the definite integral of the curve using the
        %   endpoints of the curve to define the x-axis.
        %
        %   Example 1 - By definition, a line's area is always zero.
        %       G = Geometry1D.draw('Line1D', 'Points', rand(2,2));
        %       G.area
        %
        %   Example 2 - The sign of the area depends on the orientation of the
        %               curve.
        %       G = Geometry1D.draw('Arc1D', 'Radius', 0.5, 'Angle', pi/3);
        %       G.area
        %       G = reverse(G);
        %       G.area
        %
        % See also Geometry1D
        areaOut = area(this)
        
        %coincidenceType - Returns a flag indicating if the two curves overlap
        %   flag = coincidenceType(G1,G2) returns a string indicating the
        %  	relationship between the two curves G1 and G2. The possible valid
        %  	return types are
        %       
        %       'none'       - No special relationship between the two curves
        %       'parallel'   - The curves may partially overlap
        %       'coincident' - The curves perfectly overlapping
        %
        %   Example: Test the relationships between some arcs.
        %       G1 = Geometry1D.draw('Arc', 'Radius', 1, 'Angle', pi/2);
        %       G2 = Geometry1D.draw('Arc', 'Radius', 1, 'Angle', pi*rand);
        %       G3 = Geometry1D.draw('Arc', 'Radius', 1, 'Angle', pi/3, 'Rotation', pi);
        %       G4 = Geometry1D.draw('Arc', 'Radius', 1, 'Angle', pi/2, 'Position', rand(1,2));
        %       G5 = Geometry1D.draw('Arc', 'Radius', 1, 'Angle', pi/2);
        %
        %       G1.coincidenceType(G1)
        %       G1.coincidenceType(G2)
        %       G1.coincidenceType(G3)
        %       G1.coincidenceType(G4)
        %       G1.coincidenceType(G5)
        %
        % See also Geometry1D
        flagOut = coincidenceType(this, geometryIn)
     	
        %inOn - Determine the In/On/Out status of a set of points
        %   [In,On,N,S] = inOn(G,X,Y) determines the position of a set of points
        %   relative to the curve. The vectors X and Y are a the x- and
        %   y-components, respectively, of a set of test points. The logical
        %   vector In is such that the points X(In) and Y(In) are interior to G.
        %   Similarly, the points X(On) and Y(On) are on G, with unit normals
        %   point to the interior of G given by N(On,:). The vector S is a set
        %   of parameters such that G.x(S(On)) == X(On) and G.y(S(On)) == Y(On)
        %   (within numerical tollerances). The components S(~On) are nans.
        %
        %   Example 1: Find the points which are interior to an arc and on an
        %              arc. Plot the unit normals.
        %         G = Geometry1D.draw('Arc1D', 'Radius', 1, 'Angle', pi/3);
        %         theta = linspace(-pi/6, pi/2, 10).';
        %         X = [1.25 - 2.5*rand(100,1); cos(theta)];
        %         Y = [1.25 - 2.5*rand(100,1); sin(theta)];
        %         [In, On, N] = G.inOn(X, Y);
        %         figure;hold on;axis equal
        %         G.plot;
        %         scatter(X(In), Y(In), 'b', 'o');
        %         scatter(X(~In & ~On), Y(~In & ~On), 'r', 'x');
        %         quiver(X(On), Y(On), N(On,1), N(On,2));
        %
        %   Example 2: Find the points which are interior to a line and on an
        %              line. Plot the unit normals.
        %       G           = Geometry1D.draw('Line1D', 'Points', [0 0;1 1]);
        %       XYon        = linspace(-0.5, 1.5, 10).';
        %       X           = [1 - 3*rand(100,1); XYon];
        %       Y           = [1 - 3*rand(100,1); XYon];
        %       [In, On, N] = G.inOn(X, Y);
        %       figure; hold on; axis equal
        %       G.plot;
        %       scatter(X(In), Y(In), 'b', 'o');
        %       scatter(X(~In & ~On),Y(~In & ~On), 'r', 'x');
        %       quiver(X(On), Y(On), N(On,1), N(On,2));
        %
        % See also x, y, Orientaiton, cart2s
        [In, On, N, S] = inOn(this, X, Y)
        
        %cart2s - Convert a set of points to parameter values
        %   [S,I] = cart2s(G,X,Y) calculates equivalent curve parameter values
        %   in S from the cartesian coordinates in X and Y. The vector S is a 
        %   set of parameters such that G.x(S(I)) == X(I) and G.y(S(I)) == Y(I)
        %   (within numerical tollerances). The components S(~On) are nans.
        %   The call listed above is shorthand for [~,On,~,S] = inOn(G,X,Y).
        %
        %   Example: The cart2s method serves as an inverse to the x and y
        %              methods.
        %
        %       G         = Geometry1D.draw('Line1D', 'Points', rand(2,2));
        %       s1        = linspace(0, 1);
        %       X1        = G.x(s1);
        %       Y1        = G.y(s1);
        %       [s2,isOn] = G.cart2s(X1, Y1);
        %       max(abs(s1 - s2)) < 1e-6 %beware of finite precision
        %
        % See also x, y, inOn
        [S, I] = cart2s(this, X, Y)
    end
end

function [X, Y, SIOut, SJOut] = generalIntersection(thisI, thisJ, I, J)
    %% Finds the pairwise intersection points of the curves [thisI(I),thisJ(J)]
    
    if nargin == 2
        I = 1;
        J = 1;
    end
    nIn = numel(I);
    
    if nIn > 0
        switch [class(thisI),class(thisJ)]
            case 'ArcArc'
                [XTest, YTest, isValid] = arcArcPossibleIntersectionPoints(thisI, thisJ, I, J);
            case 'PolylinePolyline'
                [XTest, YTest, isValid] = lineLinePossibleIntersectionPoints(thisI, thisJ, I, J);
            case 'ArcPolyline'
                [XTest, YTest, isValid] = arcLinePossibleIntersectionPoints(thisI, thisJ, I, J);
            case 'PolylineArc'
                [XTest, YTest, isValid] = arcLinePossibleIntersectionPoints(thisJ, thisI, J, I);
            otherwise
                error('Geometry1D:intersection', 'No intersection method implented for classes %s and %s', class(this(I)), class(this(J)));
        end
        
       	XEndI = [[thisI.X0].' , [thisI.X1].'];
        YEndI = [[thisI.Y0].' , [thisI.Y1].'];
        XEndJ = [[thisJ.X0].' , [thisJ.X1].'];
        YEndJ = [[thisJ.Y0].' , [thisJ.Y1].'];
        
        XEndI = XEndI(I,:);
        YEndI = YEndI(I,:);
        XEndJ = XEndJ(J,:);
        YEndJ = YEndJ(J,:);
              
        [SI, isOnI] = getParameter(thisI, I, [XEndJ, XTest], [YEndJ, YTest]);
        [SJ, isOnJ] = getParameter(thisJ, J, [XEndI, XTest], [YEndI, YTest]);

        %% Postprocessing
        includeEndpointJ    = isOnI(:, 1:2);
        includeEndpointI    = isOnJ(:, 1:2);
        isIntersectionPoint = isOnI(:, 3:end) & isOnJ(:, 3:end) & isValid;
        isOnI(:, 3:end)     = isIntersectionPoint;
        isOnJ(:, 3:end)     = isIntersectionPoint;
        
        if nIn == 1
            XEndI = XEndI.';
            XEndJ = XEndJ.';
            YEndI = YEndI.';
            YEndJ = YEndJ.';
            XTest = XTest.';
            YTest = YTest.';
            SI    = SI.';
            SJ    = SJ.';
        end

        X = [XEndI(includeEndpointI); XEndJ(includeEndpointJ); XTest(isIntersectionPoint)];
         
        Y = [YEndI(includeEndpointI); YEndJ(includeEndpointJ); YTest(isIntersectionPoint)];
                
        SI = SI([includeEndpointJ, isIntersectionPoint]);
        SJ = SJ([includeEndpointI, isIntersectionPoint]);

        [~, nTestPoints] = size(isOnI);
        
     	II               = I(:,ones(1, nTestPoints));
        JJ               = J(:,ones(1, nTestPoints));
        II               = II(isOnI);    	
        JJ               = JJ(isOnJ);
                 
        [nRows, ~] = size(X); 
        if nRows > 1
            XY          = [X, Y];
            XY          = sortrows(XY);
            D           = sqrt(sum(diff(XY).^2,2));
            scaleFactor = max([thisI.bbRadius, thisJ.bbRadius]);
            isUnique    = [true; D > (scaleFactor * sqrt(eps))];
            X           = XY(isUnique, 1);
            Y           = XY(isUnique, 2);
        end
        
        SIOut = cell(numel(thisI), 1);
        for i = 1:numel(thisI)
            iSelect  = (II == i);
            s        = SI(iSelect);
            if isempty(s)
                SIOut{i} = [];
            else
                SIOut{i}   = SI(iSelect);
                [nRows, ~] = size(SIOut{i});
                if nRows > 1
                    SIOut{i} = SIOut{i}.';
                end
            end
        end
        
        SJOut = cell(numel(thisJ), 1);
        for j = 1:numel(thisJ)
            jSelect  = (JJ == j);
            s        = SJ(jSelect);
            if isempty(s)
                SJOut{j} = [];
            else
                SJOut{j} = SJ(jSelect);
                [nRows, ~] = size(SJOut{j});
                if nRows > 1
                    SJOut{j} = SJOut{j}.';
                end
            end
        end
        
        if nargin == 2
            SIOut = SIOut{1};
            SJOut = SJOut{2};
            nI    = numel(SIOut);
            nJ    = numel(SJOut);
            if n1 == 0 && n2 == 0
                SIOut = [0, 0];
                SJOut = [];
            else
                SIOut = [[SIOut;zeros(nJ - nI, 1)], [SJOut;zeros(nI - nJ, 1)]];
            end
        end
    else
        X     = [];
        Y     = [];
        SIOut = cell(numel(thisI), 1);
        SJOut = cell(numel(thisJ), 1);
    end
end

function [X, Y, isValid] = lineLinePossibleIntersectionPoints(lineI, lineJ, I, J)
        %% Get Data
        nIn = numel(I);
        X   = zeros(nIn, 1);
        Y   = zeros(nIn, 1);
        
        XI  = [[lineI.X0].' , [lineI.X1].'];
        YI  = [[lineI.Y0].' , [lineI.Y1].'];    
        XJ  = [[lineJ.X0].' , [lineJ.X1].'];
        YJ  = [[lineJ.Y0].' , [lineJ.Y1].'];
        
        XI  = XI(I,:);
        YI  = YI(I,:);
        XJ  = XJ(J,:);
        YJ  = YJ(J,:);
        
        dxI = XI(:,2) - XI(:,1);
        dyI = YI(:,2) - YI(:,1);

        dxJ = XJ(:,2) - XJ(:,1);
        dyJ = YJ(:,2) - YJ(:,1);
        
        areParallel = abs(dxI.*dyJ - dxJ.*dyI) < sqrt(eps);
        isValid     = ~areParallel;
        
        %% Check for line/line intersection
        slopeI     = dyI ./ dxI;
        slopeJ     = dyJ ./ dxJ;

        isVertI    = abs(slopeI) > sqrt(1/eps);
        isVertJ    = abs(slopeJ) > sqrt(1/eps);
        notVert    = ~(isVertI|isVertJ);
        interceptI = ((YI(:,1) + YI(:,2)) - slopeI.*(XI(:,1) + XI(:,2)))/2;
        interceptJ = ((YJ(:,1) + YJ(:,2)) - slopeJ.*(XJ(:,1) + XJ(:,2)))/2;

        X(isVertI) = (XI(isVertI,1) + XI(isVertI,2))/2;
        Y(isVertI) = slopeJ(isVertI).*X(isVertI) + interceptJ(isVertI);

        X(isVertJ) = (XJ(isVertJ,1) + XJ(isVertJ,2))/2;
        Y(isVertJ) = slopeI(isVertJ).*X(isVertJ)+interceptI(isVertJ);

        X(notVert) = (interceptJ(notVert) - interceptI(notVert)) ./ (slopeI(notVert) - slopeJ(notVert));
        Y(notVert) = ((slopeI(notVert) + slopeJ(notVert)).*X(notVert) + (interceptI(notVert) + interceptJ(notVert)))/2;
end

function [X, Y, isValid] = arcArcPossibleIntersectionPoints(arcI, arcJ, I, J)
        %% Get datan
        nI    = numel(arcI);
        radI  = [arcI.Radius].';
        posI  = [arcI.Position];
        posI  = [posI(1:2:(2*nI)); posI(2:2:(2*nI))].';
        
        radI  = radI(I,:);
        posI  = posI(I,:);
        
        nJ    = numel(arcJ);
        radJ  = [arcJ.Radius].';
        posJ  = [arcJ.Position];
        posJ  = [posJ(1:2:(2*nJ)); posJ(2:2:(2*nJ))].'; 
        
        radJ  = radJ(J,:);
        posJ  = posJ(J,:);

        %% Test for intersections
        dXC = posI(:,1) - posJ(:,1);
        dYC = posI(:,2) - posJ(:,2);
        dRC = hypot(dXC, dYC);
        
        API = atan2(-dYC, -dXC);
        APJ = atan2(dYC, dXC);

        dAI = acos((radJ.^2 - dRC.^2 - radI.^2) ./ (-2*radI.*dRC));
        dAJ = acos((radI.^2 - dRC.^2 - radJ.^2) ./ (-2*radJ.*dRC));

        aTestIp = API + dAI;
        aTestIm = API - dAI;        
        aTestJm = APJ + dAJ;
        aTestJp = APJ - dAJ;
        
       	X = [(posJ(:,1) + radJ.*cos(aTestJm)) + (posI(:,1) + radI.*cos(aTestIm)),...
             (posJ(:,1) + radJ.*cos(aTestJp)) + (posI(:,1) + radI.*cos(aTestIp))]/2;
         
        Y = [(posJ(:,2) + radJ.*sin(aTestJm)) + (posI(:,2) + radI.*sin(aTestIm)),...
        	 (posJ(:,2) + radJ.*sin(aTestJp)) + (posI(:,2) + radI.*sin(aTestIp))]/2;
         
        X = real(X);
        Y = real(Y);
        
        %% The arcs cannot intersect if the have the same center
        scaleFactor = max(max(radI), max(radJ));
        isValid     = dRC  > sqrt(eps) * scaleFactor;
        isValid     = [isValid, isValid];
end

function [X, Y, isValid] = arcLinePossibleIntersectionPoints(arcI, lineJ, I, J)
    %% GET DATA
    radI = [arcI.Radius].';
    posI = [arcI.Position];
    nArc = numel(arcI);
    posI = [posI(1:2:(2*nArc)); posI(2:2:(2*nArc))].';
    
    radI = radI(I,:);
    posI = posI(I,:);
    
    xJ   = horzcat([lineJ.X0].', [lineJ.X1].');
    yJ   = horzcat([lineJ.Y0].', [lineJ.Y1].');
    
    xJ   = xJ(J,:);
    yJ   = yJ(J,:);
    
    %% TEST FOR INTERSECTION
    dxJ          = xJ(:,2) - xJ(:,1);
    dyJ          = yJ(:,2) - yJ(:,1);
    nIn          = numel(I);
    xChordCenter = zeros(nIn, 1);
    yChordCenter = zeros(nIn, 1);
    
    isVert               = (abs(dxJ) < sqrt(eps) * abs(dyJ));
    xChordCenter(isVert) = mean(xJ(isVert,:), 2);
    yChordCenter(isVert) = posI(isVert, 2);
    
    isHorz               = (abs(dyJ) < sqrt(eps) * abs(dxJ));
    xChordCenter(isHorz) = posI(isHorz, 1);
    yChordCenter(isHorz) = mean(yJ(isHorz, :), 2);
    
    %% Find the mid-point of the chord of the arc coincident with the line
    isNorm = ~(isVert | isHorz);
    if any(isNorm)
        lineSlope = dyJ(isNorm) ./ dxJ(isNorm);
        lineB     = (yJ(isNorm,1) + yJ(isNorm,2) - lineSlope.*xJ(isNorm,1) - lineSlope.*xJ(isNorm,2)) / 2;

        arcSlope             = -dxJ(isNorm) ./ dyJ(isNorm);
        arcB                 = posI(isNorm, 2) - (arcSlope.*posI(isNorm, 1));
        
        xChordCenter(isNorm) = (arcB - lineB) ./ (lineSlope - arcSlope);
        yChordCenter(isNorm) = (lineSlope .* xChordCenter(isNorm) + lineB)/2 + (arcSlope  .* xChordCenter(isNorm) + arcB)/2;
    else
        lineSlope = 0;
    end
    
    %% Calculate the length of this chord
    deltaTangent        = hypot(xChordCenter - posI(:,1), yChordCenter-posI(:,2));
    chordLength         = 2 * sqrt(radI.^2 - deltaTangent.^2);
    isZero              = abs(chordLength) < sqrt(eps);
    chordLength(isZero) = 0;
    
    %% If an imaginary length, the line and the arc do not intersect
    isImag              = abs(imag(chordLength)) > sqrt(eps);
    isImag              = [isImag, isImag];
    isValid             = ~isImag;
    
    %% Otherwise the possible intersections are at the endpoints of the chord
    chordLength         = real(chordLength);
    dxChord             = zeros(nIn, 1);
    dyChord             = zeros(nIn, 1);
    dxChord(isNorm)     = chordLength(isNorm) ./ (sqrt(1 + lineSlope.^2)) / 2;
    dyChord(isNorm)     = lineSlope .* dxChord(isNorm);
    dxChord(isVert)     = 0;
    dxChord(isHorz)     = chordLength(isHorz) / 2;
    dyChord(isVert)     = chordLength(isVert) / 2;
    dyChord(isHorz)     = 0;
    
    X                   = [xChordCenter + dxChord, xChordCenter - dxChord];
    Y                   = [yChordCenter + dyChord, yChordCenter - dyChord];
end