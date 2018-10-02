classdef Region2D < Component    
    properties
        Geometry
        Material
        Dynamics
    end
    
    methods
        %% Constructor Method
        function this = Region2D(varargin)
            if nargin~=0
                for i = 1:2:nargin
                    this.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        %% Setters
        function this = set.Dynamics(this, value)
            if isa(value, 'DynamicsTypes')
             	this.Dynamics = value;
            elseif ischar(value)
                this.Dynamics = DynamicsTypes.(value);
            else
                this.Dynamics = DynamicsTypes(value);
            end
        end
        
        %% Visualization Methods
        function gHandleOut = plot(this)
            gHandleOut = plot([this.Geometry]);
        end
        
        function gHandleOut = wireframe(this)
            gHandleOut = wireframe([this.Geometry]);
        end
        
        function this       = rotate(this, rotation, position)
            rotate([this.Geometry], rotation, position);
            [this.Material] = rotate([this.Material], rotation, position);
        end
        
        function this       = reversePolarity(this)
            [this.Material] = reversePolarity([this.Material]);
        end

        %% Utility Methods
        function boolOut = hasPolarMirrorSymmetry(this)
            nRegions = numel(this);
          	boolOut  = false;
            if nRegions > 0
                x = [];
                y = [];
                s = linspace(0,1).';
                
                for iRegion = 1:nRegions
                    nCurves = numel(this(iRegion).Geometry.Curves);
                    for iCurve = 1:nCurves
                        x = [x;this(iRegion).Geometry.Curves(iCurve).x(s)];
                        y = [y;this(iRegion).Geometry.Curves(iCurve).y(s)];
                    end
                end
                
                %determine typical size of geometry
                scaleFactor = max(max(x)-min(x),max(y)-min(y));
                
                %rotate to positive x axis
                r = hypot(x,y);
                a = atan2(y,x);
                a = a  - (max(a)+min(a)) / 2;
                
                x = r.*cos(a);
                y = r.*sin(a);
                
                %truncate the precision to avoid numerical errors
                x = single(x);
                y = single(y);
                
                %remove points on the x axis, which is an axis of symmetry
                offXAxis = abs(y) > eps * scaleFactor;
                
                x = x(offXAxis);
                y = y(offXAxis);
                
                %number of points in the upper and lower plane should be equal
                upperHalf = y > eps * scaleFactor;
                lowerHalf = ~upperHalf;
                
                if sum(upperHalf) == sum(lowerHalf)
                    xp    = x(upperHalf);
                    yp    = y(upperHalf);

                    xl    = x(lowerHalf);
                    yl    = y(lowerHalf);

                    [~,Ip] = sortrows([xp,yp],[2 1]);
                    [~,Il] = sortrows([xl,yl],[-2 1]);

                    xp     = xp(Ip);
                    yp     = yp(Ip);

                    xl     = xl(Il);
                    yl     = yl(Il);

                    d      = sqrt((xp-xl).^2+(yp+yl).^2);
                    if max(d) == 0
                        boolOut = true;
                    end
                else
                    boolOut = false;
                end
            else
                boolOut = true;
            end
        end
    end
    
    methods (Access = protected)
        function copyObj = copyElement(this)
            copyObj          = copyElement@Component(this);
            copyObj.Geometry = copy(copyObj.Geometry);
        end
    end
end