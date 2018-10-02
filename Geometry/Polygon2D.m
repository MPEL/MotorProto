classdef (Sealed) Polygon2D < Geometry2D
  	%Polygon2D.m Creates an object representing a two dimensional polygon
    % G = Polygon2D('PropertyName', propertyvalue,...) creates an object
    % representing a two dimensional polygon.
    %
    %
    % Polygon2D methods:
    %   Polygon2D defines no new methods
    %
    % Polygon2D properties:
    %   Points - An n by 2 matrix representing the vertices of the polygon
    %
    % Polygon2D inhertance:
    %   Polygon2D inherts methods and properties from Geometry2D. See the help 
    %   for Geometry2D for more information.
    %
    % See also Geometry2D, Geometry, Parameterizable, MotorProto
    
    %% Public Properties
    properties (SetAccess=protected)
        Points
    end
    
    %% Public Methods
    methods
        %% Constructor Method
        function this = Polygon2D(varargin)
            this = this@Geometry2D;
            %% Build Curves
            if nargin~=0               	
                nVarargin = numel(varargin);
                for i = 1:2:nVarargin
                    this.(varargin{i}) = varargin{i+1};
                end
                this = build(this);
            end
        end
        
        function this = build(this)
            %% get parameters
            points   = this.Points;
            position = this.Position;
            rotation = this.Rotation;
            orient   = this.Orientation;
            
            %% rotate
            rotationMatrix = [ cos(rotation) sin(rotation);
                              -sin(rotation) cos(rotation)];
            points         = points * rotationMatrix;

            %% shift
            points(:,1) = points(:,1) + position(1);
            points(:,2) = points(:,2) + position(2);
            
            %% make Curves
            [nPoints,~] = size(points);
            curveArray  = Polyline.empty(0, 1);
            for i = 1:nPoints-1
                curveArray(i) = Geometry1D.draw('Polyline', 'Points', points([i, i+1], :), 'Orientation', orient);
            end
            curveArray(nPoints)	= Geometry1D.draw('Polyline', 'Points', points([nPoints, 1], :), 'Orientation', orient);
            this.Curves = curveArray;
            this        = sortCurves(this);
        end
    end
end