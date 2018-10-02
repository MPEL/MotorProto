classdef Point < Geometry0D
    %%Point An object representing a point or set of points
    %   Point objects represent sets of points in the problem domain. They may
    %   be used to fix certain node locations in the resulting finite element
    %   mesh.
    %
    % Point properties:
    %   X - An array of X coordinates
    %   Y - An array of Y coordinates
    %
    % Point inheritance:
    %   Point inherits all methods and properties of Geometry0D. Refer to the
    %   help for Geometry0D for more details.
    %
    % See also Geometry0D, Geometry, Parameterizable, MotorProto

%{
properties:
    %X - The X coordinate of the point
    %   This property represent the X coordinate (or set of coordinates) for the
    %	point object. It is parameterizable.
    %
    % See also Parameterizable
    X;
    
    %Y - The Y coordinate of the point
    %   This property represent the Y coordinate (or set of coordinates) for the
    %	point object. It is parameterizable.
    %
    % See also Parameterizable
    Y;
%}        
    %% Public Properties
    properties
        X
        Y
    end
    
    %% Public Methods
    methods
        %% Constructor
        function this = Point(varargin)
            this = this@Geometry0D(varargin{:});
        end
    end
end