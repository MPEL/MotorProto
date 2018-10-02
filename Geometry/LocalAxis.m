classdef LocalAxis
    properties (SetAccess = protected)
        InitialPosition = [0, 0];
        InitialAngle    = 0;
        RotationAngle   = 0;
        RotationAxis    = [0, 0];
    end
    
    properties (Dependent)
        Rotation
        Position
    end  
    
    properties
        Rotation_ = 0
        Position_ = [0, 0]
    end
    
    properties (Dependent)
        Angle
    end
    
    methods
        %% Constructor Method
        function this = LocalAxis(varargin)
            if nargin~=0
                nVarargin = numel(varargin);
                for i = 1:2:nVarargin
                    this.(varargin{i}) = varargin{i+1};
                end
                this.Position  = this.calculatePosition;
                this.Rotation  = this.calculateRotation;
            end
        end
        
        function this = set.Rotation(this, rotationAngle)
        	this.InitialAngle  = rotationAngle;
            this.RotationAngle = 0;
         	this.Rotation_     = calculateRotation(this);
        end
        
        function angleOut = get.Rotation(this)
            angleOut = this.Rotation_;
        end
        
        function this = set.Position(this, rotationAxis)
        	this.InitialPosition = rotationAxis;
            this.RotationAxis    = [NaN, NaN];
            this.Position_       = calculatePosition(this);
        end
        
        function coordOut = get.Position(this)
            coordOut = this.Position_;
        end
        
        function angleOut = get.Angle(this)
            angleOut = this.Rotation;
        end
        
        %% Cache Methods
        function angleOut = calculateRotation(this)
            angleOut = this.InitialAngle + sum([this.RotationAngle]);
            angleOut = mod(angleOut + pi, 2*pi) - pi;
        end
        
        function positionOut = calculatePosition(this)
            positionOut   = this.InitialPosition;
            rotationAngle = [this.RotationAngle];
            nRotations    = numel(rotationAngle);
            rotationAxis  = reshape([this.RotationAxis], [], nRotations).';
            for iRotation = 1:nRotations
                currentAxis    = rotationAxis(iRotation, :);
                if any(isnan(currentAxis))
                    currentAxis = positionOut;
                end
                currentAngle   = rotationAngle(iRotation);
                rotationMatrix = [cos(currentAngle) sin(currentAngle); -sin(currentAngle) cos(currentAngle)];
                positionOut    = currentAxis + (positionOut - currentAxis)*rotationMatrix;
            end
        end
        
        %% Rotation Method
        function varargout = rotate(varargin)
            this    = [varargin{1:(end-2)}];
            angleIn = varargin{end-1};
            axisIn  = varargin{end};
            
            nThis    = numel(this);
            thisSize = numel(this);
            
            nAngles             = numel(angleIn);
            newAngle(1,nAngles) = angleIn(end);
            if nAngles == 1
                newAngle = repmat(newAngle, 1, thisSize);
            else
                for iAngle = 1:(nAngles - 1)
                    newAngle(1,iAngle) = angleIn(iAngle);
                end
            end
            angleIn = newAngle;
            newAngle = num2cell(newAngle);
            
            nAxis = numel(axisIn) / 2;
            if nAxis == 1
                newAxis = repmat(axisIn, thisSize, 1);
            else
                newAxis = zeros(nAxis,2);
                for iAngle = 1:(nAxis - 1)
                    newAxis(1,iAngle) = axisIn(iAngle,:);
                end
                newAxis(end,:) = axisIn(end,:);
            end
            axisIn  = newAxis;
            newAxis = mat2cell(newAxis, ones(thisSize,1), 2).';
            
            %%
            newRotationAngle     = {this.RotationAngle};
            newRotationAngle     = cellfun(@(x, y)(vertcat(x, y)), newRotationAngle, newAngle, 'UniformOutput', false);
            [this.RotationAngle] = deal(newRotationAngle{:});            
                
            newRotationAxis     = {this.RotationAxis};
            newRotationAxis     = cellfun(@(x, y)(vertcat(x, y)), newRotationAxis, newAxis, 'UniformOutput', false);
            [this.RotationAxis] = deal(newRotationAxis{:});
            
         	%% Calculate the new angle
            newRotation      = [this.Rotation_];
            newRotation      = newRotation + angleIn;
            newRotation      = mod(newRotation+pi, 2*pi) - pi;
            newRotation      = num2cell(newRotation);
            [this.Rotation_] = deal(newRotation{:});
            
            %% Calculate the new position
            newPosition            = [this.Position_];
            newPosition            = reshape(newPosition, 2, []).';
            
            keepPosition           = any(isnan(axisIn), 2);
            axisIn(keepPosition,:) = newPosition(keepPosition,:);
            newPosition            = newPosition - axisIn;
            newPosition            = [sum(newPosition.*[cos(angleIn.'), -sin(angleIn.')], 2), sum(newPosition.*[sin(angleIn.'), cos(angleIn.')], 2)];
            newPosition            = newPosition + axisIn;
            newPosition            = mat2cell(newPosition,ones(nThis, 1), 2).';
            [this.Position_]       = deal(newPosition{:});
            
            if nargout == 1
                varargout = {this};
            elseif nargout == nThis
                varargout = num2cell(this); 
            else
                varargout = [num2cell(this), newRotation, newPosition];
            end
        end
    end
end