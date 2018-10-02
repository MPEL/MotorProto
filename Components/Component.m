classdef Component < Nameable
    properties (Hidden)
        IsUserDefined = true;
    end

    methods
     	function this = Component(nameIn, varargin)
            if nargin ~= 0
                this.Name = nameIn;
                for i = 1:2:(nargin-1)
                    this.(varargin{i}) = varargin{i+1};
                end
            end
        end
    end
    
    methods (Static)
        function componentOut = newComponent(componentType, varargin)
            componentOut = eval(componentType);
            if isa(componentOut, 'Component')
                componentOut = componentOut.newComponent(varargin{:});
            else
                error('MotorProto:Component', '%s is not a recognized Component subclass', componentType);
            end
        end
    end
end