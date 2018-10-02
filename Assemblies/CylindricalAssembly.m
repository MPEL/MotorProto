classdef CylindricalAssembly < Assembly
    %CylindricalAssembly.m An abstract class representing Assembly objects defined on a cylindrical annulus
    %   Subclasses of the CylindricalAssembly class represent Assembly objects
    %   which have a DomainHull defined on a cylindrical annulus.
    %
    % CylindricalAssembly properties:
    %   InnerRadius - The inner radius of the CylindricalAssembly
    %   OuterRadius - The outer radius of the CylindricalAssembly
    %   Length      - The length of the CylindricalAssembly
    %
    % CylindricalAssembly inherits properties and methods from Assembly.
    %
    % See the help file for Assembly for more information.
    %
    % See also MotorProto, Model, Assembly
    
%{
properties:
 	%InnerRadius - The inner radius of the CylindricalAssembly object
    %
    % See also CylindricalAssembly
    InnerRadius;
    
 	%OuterRadius - The outer radius of the CylindricalAssembly object
    %
    % See also CylindricalAssembly
    OuterRadius;
    
 	%Length - The length of the CylindricalAssembly object
    %
    % See also CylindricalAssembly
    Length;
%}
    properties
        %% Basic Geometry Properties
        InnerRadius = 0;
        OuterRadius = 1;
        Length      = 1;
    end
    
	properties (Abstract,Dependent)
        %% Symmetry Properties
        SpatialSymmetries
        HasHalfWaveSymmetry
        SpaceTimeSymmetries
        GeometricSymmetries
        
        GeometryMirrorSymmetry
    end
    
    properties (Dependent, SetAccess = private)
        Mass
        TangentialBoundaries
    end
    
    methods
        %% Constructions
     	function this = CylindricalAssembly(varargin)
            this = this@Assembly(varargin{:});
        end
        
        %% Getters
        function angles = get.TangentialBoundaries(this)
            if ~isempty(this.DomainHull)
                da     = this.DomainHull.Angle;
                a      = this.DomainHull.Rotation;
                angles = [a,a + da];
            else
                angles = [];
            end
        end
        
        function mass = get.Mass(this)
            regions  = this.Regions;
            nRegions = length(regions);
            mass     = 0;
            for i = 1:nRegions
                mass = mass + regions(i).Geometry.area * regions(i).Material.Density;
            end
            mass = mass * this.Length;
            mass = mass / this.ModeledFraction;
        end
        
        %% Element Functions
        function previewElement(this)
            dh = makeElementDomainHull(this);
            
            if ~isempty(this.InputRegions);
                inputGeometry = [this.InputRegions.Geometry];
                dh = dh - inputGeometry;
            else
                inputGeometry = [];
            end
            
            plot([dh, inputGeometry]);
        end
        
        function hull = makeElementDomainHull(this)
            r    = [this.InnerRadius, this.OuterRadius];
            a    = 2 * pi / this.GeometricSymmetries;
            hull = Geometry2D.draw('Sector', 'Radius', r, 'Angle', a, 'Rotation', -a/2);
        end
    end
end