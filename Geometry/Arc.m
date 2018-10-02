classdef (Sealed) Arc < Geometry1D
    %Arc.m Creates an object representing a 1-dimensional arc
    %   G = Arc('PropertyName',propertyvalue,...) creates an object representing
    %   a 1-dimensional arc.
    %
    % Arc properties:
    %   Radius - The radius of the arc
    %   Angle  - The angle subtended by the arc
    %
    % Arc methods:
    %   Arc defines no new methods.
    %
    % Arc inheritance:
    %   Arc inherts methods and properties directly from Geometry1D. See the
    %   help for Geometry1D for more information.
    %
    %   See also Geometry1D, Geometry, Parameterizable, MotorProto
    
    properties (Dependent)
        Order
    end
    
    properties
        Radius = 0;
        Angle  = 0;
        Base   = 'center'
    end

    methods
        function this = Arc(varargin)
            this = this@Geometry1D;
            if nargin ~= 0
                if mod(nargin, 2) == 0                    
                    for iArg = 1:2:nargin
                        this.(varargin{iArg}) = varargin{iArg + 1};
                    end
                else
                    thisSize          = varargin{1};
                    this(1, thisSize) = this;
                    this(:)           = this(1, end);
                    this              = copy(this);
                    for iArg = 2:2:nargin
                        if ~iscell(varargin{iArg+1})
                            varargin{iArg + 1} = num2cell(varargin{iArg + 1});
                        end
                        [this.(varargin{iArg})] = deal(varargin{iArg + 1}{:});
                    end
                end
                this.updateCache;
            end
        end
        
        %% Setters
        function this = set.Radius(this, radiusIn)
            this.Radius = radiusIn;
          	this        = updateCache(this);
        end
        
        function this = set.Angle(this, angleIn)
            this.Angle = angleIn;
          	this       = updateCache(this);
        end
        
        function this = set.Base(this, baseIn)
            this.Base = baseIn;
            this      = updateCache(this);
        end
        
        function orderOut = get.Order(~)
            orderOut = 2;
        end
        
        function xOut = x(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius   = this.Radius;
            position = this.Position;
            
            xOut = position(1) + radius*cos(rotation + angle*s);
        end
        
        function yOut = y(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius   = this.Radius;
            position = this.Position;
            
            yOut = position(2) + radius*sin(rotation + angle*s);
        end
        
        function dxOut = dx(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius = this.Radius;
            
            dxOut = -angle*radius*sin(rotation + angle*s);
        end
        
        function dyOut = dy(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius = this.Radius;
            
            dyOut = angle*radius*cos(rotation + angle*s);
        end
        
        function lengthOut = elementLength(this)
            radius    = this.Radius;
            angle     = this.Angle;
            lengthOut = abs(radius*angle);
        end
        
        function nOut = elementMinEdgeNumber(this)
            nOut = ceil(14*abs(this.Angle) / (2*pi));
        end
        
        function dx2Out	= dx2(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius = this.Radius;
            
            dx2Out = -angle^2*radius*cos(rotation + angle*s);
        end
        
        function dy2Out	= dy2(this, s)
            switch this.Orientation
                case true
                    angle    = this.Angle;
                    rotation = this.Rotation;
                case false
                    angle    = -this.Angle;
                    rotation = this.Rotation - angle;
            end
            radius = this.Radius;
            
            dy2Out = -angle^2*radius*sin(rotation + angle*s);
        end
        
        function areaOut = area(this)
            angle 	= this.Angle;
            radius 	= this.Radius;
            areaOut = radius^2 * (angle - sin(angle))/2;
            if ~this.Orientation
                areaOut = -areaOut;
            end
        end
        
        function arcsOut = split(this, sParams)
            [nRows, nCols] = size(sParams);
            if nRows > 1
                sParams = sParams.';
                nCols   = nRows;
            end
            
            [nRows, nCols] = size(this);
            if nRows > 1
                this  = this.';
                nCols = nRows;
            end
            
            %% Only split where necessary
            toCopy  = cellfun('isempty', sParams);
            toSplit = ~toCopy;
            
            if any(toSplit)
                thisCopy  = this(toCopy);
                thisSplit = this(toSplit);
                sParams   = sParams(toSplit);

                %% Get Original Properties
                radii    = [thisSplit.Radius];
                angle    = [thisSplit.Angle];
                rotation = [thisSplit.Rotation];
                position = [thisSplit.Position];
                position = reshape(position, 2, []).';
                base     = {thisSplit.Base};
                orient   = [thisSplit.Orientation];

                %% Calculate Parameters
                sParams  = cellfun(@(x)(horzcat(0, x)), sParams, 'UniformOutput', false);
                dsParams = cellfun(@(x)(diff(horzcat(x, 1))), sParams, 'UniformOutput', false);

                sDims    = cellfun('length', sParams);
                nNewArcs = sum(sDims);

                sParams  = cell2mat(sParams);
                dsParams = cell2mat(dsParams);

                %%Radius
                newRadii = arrayfun(@(x, y)(repmat(x, 1, y)), radii, sDims, 'UniformOutput', false);
                newRadii = cell2mat(newRadii);

                %%Orientation
                newOrientation = arrayfun(@(x, y)(repmat(x, 1, y)), orient, sDims, 'UniformOutput', false);
                newOrientation = cell2mat(newOrientation);

                %%Position
                newPosition       = zeros(nNewArcs,2);
                newPosition(:, 1) = cell2mat(arrayfun(@(x, y)(repmat(x, y, 1)), position(:, 1), sDims.', 'UniformOutput', false));
                newPosition(:, 2) = cell2mat(arrayfun(@(x, y)(repmat(x, y, 1)), position(:, 2), sDims.', 'UniformOutput', false));
                newPosition       = num2cell(newPosition, 2);

                %%Base
                newBase = arrayfun(@(x, y)(repmat(x, 1, y)), base, sDims, 'UniformOutput', false);
                newBase = [newBase{:}];

                %%Rotation and Angle
                newRotation = arrayfun(@(x, y)(repmat(x, 1, y)), rotation, sDims, 'UniformOutput', false);
                newRotation = cell2mat(newRotation);

                newAngle    = arrayfun(@(x, y)(repmat(x, 1, y)), angle, sDims, 'UniformOutput', false);
                newAngle    = cell2mat(newAngle);

                newRotation = newRotation + newAngle.*(sParams);

                newAngle    = newAngle.*(dsParams);

                arcsOut     = [copy(thisCopy), Arc(nNewArcs, 'Radius', newRadii, 'Base', newBase, 'Position', newPosition, 'Angle', newAngle, 'Rotation', newRotation, 'Orientation', newOrientation)];
            else
                arcsOut = copy(this);
            end
        end
        
        function flagOut = coincidenceType(this, geometryIn)
            flagOut = 'none';
            
           	isSameClass = strcmp(class(geometryIn),'Arc');
            if isSameClass
                radius1 = this.Radius;
                radius2 = geometryIn.Radius;
                hasSameRadius = abs(radius1 - radius2) < sqrt(eps);
                
                if hasSameRadius
                    position1 = this.Position;
                    position2 = geometryIn.Position;
                    hasSamePosition	= norm(position1 - position2) < sqrt(eps);
                    
                    if hasSamePosition
                        rotation1 = this.Rotation;
                        rotation2 = geometryIn.Rotation;
                        angle1    = this.Angle;
                        angle2    = geometryIn.Angle;
                        
                        A1        = [rotation1, rotation1 + angle1];
                        A2        = [rotation2, rotation2 + angle2];
                        
                        hasSameAngle =  all( abs(       A1  - A2) < sqrt(eps))...
                                      | all( abs(fliplr(A1) - A2) < sqrt(eps));
                        
                      	if hasSameAngle
                            flagOut = 'coincident';
                        else
                            flagOut = 'parallel';
                        end
                    end
                end
            end
        end
         
        function boolOut = isCounterClockwise(this)
            ang     = this.Angle;
            boolOut =   ((sign(ang) ==  1) &  this.Orientation)...
                   	  | ((sign(ang) == -1) & ~this.Orientation);
        end
        
        function [W, On, Nx, Ny ,S] = inOn(this, xIn, yIn)
            %% Get properties
            radius   = this.Radius;
            angle    = this.Angle;
            position = this.Position;
            rotation = this.Rotation;
            
            %% Express points in the complex plane
            zCenter = position(1) + 1i*position(2);
            z0      = this.X0 + 1i*this.Y0;
            z1      = this.X1 + 1i*this.Y1;
            zMid    = (z0 + z1)/2;
            zIn     = xIn + 1i*yIn;
            
            %% Adjust for orientation
            if isCounterClockwise(this);
                deltaZ       = z0 - z1;
                poleIntegral = 2*pi*1i;
            else
                deltaZ       = z1 - z0;
                poleIntegral = -2*pi*1i;
            end
            
            %% Calculate vector normal to the line between the arc endpoints
            zTangent = deltaZ / abs(deltaZ); 
          	zNormal  = 1i * zTangent;
            
            %% Find points that are in the closed arc
            inCircle = abs(zCenter - zIn) < radius;
            if abs(deltaZ) < sqrt(eps)
                inClosure = true(size(zIn));
            else
                dZ             = zIn - zMid;
              	zNInnerProduct = dZ*conj(zNormal);
               	inClosure      = real(zNInnerProduct) >= 0;
            end
            
            containsPole = inClosure & inCircle;
            %% Perform Integration
            W               = log((z1 - zIn)./(z0 - zIn));
            W(containsPole) = W(containsPole) + poleIntegral;
            W               = W / (2*pi*1i);
            
            %% Shift inputs to origin
            xShifted = xIn - position(1);
            yShifted = yIn - position(2);
            
            radiusIn = hypot(xShifted, yShifted);
            angleIn  = atan2(yShifted, xShifted);
            
            %% determine if the points lie within the angle of the arc
            if angle > 0
                if rotation+angle > 0
                    needsMod          = angleIn < (rotation - sqrt(eps));
                    angleIn(needsMod) = angleIn(needsMod) + 2*pi;
                    onAngle           = angleIn < (rotation + angle + sqrt(eps));
                else
                    needsMod          = angleIn > 0;
                    angleIn(needsMod) = angleIn(needsMod) - 2*pi;
                    onAngle           =   (angleIn > (rotation - sqrt(eps))) ...
                                        & (angleIn < (rotation + angle + sqrt(eps)));
                end
            elseif angle < 0
                if rotation+angle < 0
                    needsMod          = angleIn > (rotation + sqrt(eps));
                    angleIn(needsMod) = angleIn(needsMod) - 2*pi;
                    onAngle           = angleIn > (rotation + angle - sqrt(eps));
                else
                    needsMod          = angleIn < 0;
                    angleIn(needsMod) = angleIn(needsMod) + 2*pi;
                    onAngle           =   (angleIn < (rotation + sqrt(eps))) ...
                                        & (angleIn > (rotation + angle - sqrt(eps)));
                end
            else
                error('Arc:inOn','Arc angle must not be zero');
            end
            
            %% determine if the points are inside or outside the arc radius
            if isCounterClockwise(this)
                onRadius = abs(radiusIn - radius) < sqrt(eps);
                On       = onRadius & onAngle;
                Nx       = -cos(angleIn);
                Ny       = -sin(angleIn);
            else
                onRadius = abs(radiusIn - radius) < sqrt(eps);
                On       = onRadius & onAngle;
                Nx       = cos(angleIn);
                Ny       = sin(angleIn);
            end
            
          	Nx(~On) = 0;
        	Ny(~On) = 0;
            
            %% calculate S parameter
            S           = (angleIn - rotation) / angle;
            
            %% snap to end points to avoid numerical difficulties
            nearZero    = abs(S)     < sqrt(eps);
            nearOne     = abs(S - 1) < sqrt(eps);
            
            S(nearZero) = 0;
            S(nearOne)  = 1;
        end
        
        function [S, On] = cart2s(this, xIn, yIn)
            [S, On] = this.getParameter(1, xIn, yIn);
        end
        
        function [S, On] = getParameter(this, I, xIn, yIn)
            %% Get Data
            nThis = numel(this);
            rad   = [this.Radius].';
            pos   = [this.Position];
            pos   = [pos(1:2:(2*nThis)); pos(2:2:(2*nThis))].';
            ang   = [this.Angle].';
            rot   = [this.Rotation].';
            
            rad   = rad(I,:);
            pos   = pos(I,:);
            ang   = ang(I,:);
            rot   = rot(I,:);
            
         	%% Shift and rotate inputs
            xShifted = bsxfun(@minus, xIn, pos(:,1));
            yShifted = bsxfun(@minus, yIn, pos(:,2));

            radiusIn = hypot(xShifted, yShifted);
            angleIn  = atan2(yShifted, xShifted);

            minAngle = min(rot, rot + ang);
            maxAngle = max(rot, rot + ang);
            angleIn  = bsxfun(@minus, angleIn, minAngle);
            angleIn  = angleIn + sqrt(eps)*2*pi;
            angleIn  = mod(angleIn, 2*pi);
            angleIn  = bsxfun(@plus, angleIn, minAngle);
            angleIn  = angleIn - sqrt(eps)*2*pi;

            On       = bsxfun(@lt, angleIn, maxAngle + sqrt(eps) * 2*pi);
            dR       = bsxfun(@minus, radiusIn, rad);
            onRadius = abs(dR) < sqrt(eps)*max(max(radiusIn));

            S        = bsxfun(@minus, angleIn, rot);
            S        = bsxfun(@rdivide, S, ang);
            On       = On & onRadius;

            %% snap to end points to avoid numerical difficulties
            nearZero    = abs(S)     < sqrt(eps);
            nearOne     = abs(S - 1) < sqrt(eps);

            S(nearZero) = 0;
            S(nearOne)  = 1;

            On          = On & (S >= 0) & (S <= 1);
        end
    end
    
    methods (Access = protected)
        function this = updateCache(this)
            nThis =  numel(this);
            for iThis = 1:nThis
                xCache = this(iThis).x([0, 0.5, 1]);
                yCache = this(iThis).y([0, 0.5, 1]);

                this(iThis).X0 = xCache(1);
                this(iThis).Xm = xCache(2);
                this(iThis).X1 = xCache(3);

                this(iThis).Y0 = yCache(1);
                this(iThis).Ym = yCache(2);
                this(iThis).Y1 = yCache(3);

                if abs(this(iThis).Angle) > pi*(1-sqrt(eps))
                    this(iThis).bbRadius = this(iThis).Radius;
                    this(iThis).bbCenter = this(iThis).Position;
                else
                    this(iThis).bbRadius = hypot(xCache(3) - xCache(1), yCache(3) - yCache(1));

                    this(iThis).bbCenter = [xCache(1) + xCache(3), yCache(1) + yCache(3)] / 2;
                end
            end
        end
    end
end
