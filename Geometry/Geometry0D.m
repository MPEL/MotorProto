classdef Geometry0D < Geometry
    %Geometry0D.m An abstract interface class for 0-D geometric objects.
    %
    % Geometry0D methods:
    %   plot        - Creates a scatter plot
    %   wireframe   - Creates a scatter plot
    %   draw        - Creates a new set of points
    %
    % Geometry0D properties:
    %   Dimension   - 0, The spatial dimension of a point
    %   PlotStyle   - A cell array controling the plot of the points
    %
 	% Geometry0D inheritanc:
    %   Geometry0D inherts methods and properties from Geometry. Refer to the 
    %   help for Geometry for more details.
    %
    % See also Point, Geometry, Parameterizable, MotorProto
    
%{
properties:    
    %Dimension - 0, The spatial dimension of a point
    %   This property indicates the object is 0-Dimensional (a point). All 
    %   objects derived from Geometry0D this class have the same value for this
    %	property.
    Dimension;
    
    %PlotStyle - A cell array controling how the object is plotted
    %   The PlotStyle property is a cell array containing arguments which are
    %   valid with MATLAB's scatter function.
    %
    %   Example: Edit the PlotStyle property of a set of points.
    %       G = Geometry0D.draw('Point0D', 'X',rand(10,1), 'Y',rand(10,1), 'PlotStyle',{'r','x'});                        
    %       figure;subplot(1,2,1);
    %       G.plot;
    %
    %       G.PlotStyle = {'b','o'};
    %       subplot(1,2,2);
    %       G.wireframe;
    %
	% See also scatter, Geometry0D
    PlotStyle;
%}    

    %% Public Properties
    properties (Constant = true)
        Dimension = 0;
    end
    
 	properties
        PlotStyle   = {'Marker', 'o'};
    end
    
    properties (Abstract)
       	X
        Y
    end
    
    %% Public Methods
    methods
        %% Constructor
        function this = Geometry0D(varargin)
            this = this@Geometry(varargin{:});
        end

        %% Visualization Methods
      	function gHandleOut = plot(this)
            %%plot - Creates a scatter plot
            %   plot(G) creates a scatter plot of the points defined by
            %   the X and Y properties of G. The PlotStyle property must be
            %   consistent with the usage of the MATLAB scatter funciton.
            %
            %   Example: Plot a Point object.
            %       G = Geometry0D.draw('Point', 'X',rand(1), 'Y',rand(1), 'PlotStyle',{'x','r'});
            %       G.plot;
            %
            % See also scatter, Geometry0D
            
            gHandleOut = scatter(this.X, this.Y, this.PlotStyle{:});
        end
        
        function gHandleOut = wireframe(this)
            %%wireframe - Creates a scatter plot
            %   wireframe(G) creates a scatter plot of the points defined by 
            %   the X and Y properties of G. The PlotStyle property must be 
            %   consistent with the usage of the MATLAB scatter funciton.
            %
            %   Example: Plot a Point object.
            %       G = Geometry0D.draw('Point', 'X',rand(1), 'Y',rand(1), 'PlotStyle',{'x','r'});
            %       G.wireframe;
            %
            % See also scatter, Geometry0D
            
            gHandleOut = scatter(this.X, this.Y, this.PlotStyle{:});
        end
    end
    
    methods (Static)
        %% Factory Methods
        function geometryOut = draw(typeIn, varargin)
            %%draw - Instantiates a new 0-Dimensional object
            %   G = Geometry0D.draw(typeIn,property1,value1,...) creates an
            %   object of class typeIn with the given property values.
            %
            %   Example: Create a Point object.
            %       G = Geometry0D.draw('Point', 'X',rand(1), 'Y',rand(1), 'PlotStyle',{'x','r'});
            %       G.plot;
            %
            % See also Point, Geometry0D, Geometry, Parameterizable, MotorProto
            
            switch typeIn
                case {'Point', 'point', 'Point0D', 'Point0d', 'point0D', 'point0d'}
                    geometryOut = Point(varargin{:});
                otherwise
                    error('Geometry0D:draw', 'Unknown Geometry0D subclass %s', typeIn);
            end
        end
    end
end