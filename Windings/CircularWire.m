classdef CircularWire < Wire
    properties
        ConductorDiameter   = 0;
        InsulationThickness = 0;
    end
    
    methods
        function [conductors, nonConductors, locationMatrix] = build(this, slotShape, conductorBoundaries, nTurns, nLayers, conductorDynamics, couplingType, windingType, slotAngle, airgapLoc, label)
            switch windingType
                case WindingTypes.Distributed
                    [conductors, nonConductors, locationMatrix] = buildDistributedWinding(this, slotShape, conductorBoundaries, nTurns, conductorDynamics, airgapLoc, label);
                case WindingTypes.Concentrated
                  	[conductors, nonConductors, locationMatrix] = buildConcentratedWinding(this, slotShape, nTurns, nLayers, conductorDynamics, slotAngle, airgapLoc, label);
                otherwise
                    error('Unknown winding type %s',char(windingType));
            end
        end

    	function [conductors, nonConductors, locationMatrix] = buildDistributedWinding(this, slotShape, conductorBoundaries, nTurns, conductorDynamics, airgapLoc, label)
            %% Get parameters
            conductorDiameter   = this.ConductorDiameter;
            insulationThickness = this.InsulationThickness;
            rConductor          = conductorDiameter / 2 + insulationThickness;
            
            %% Calculate x-coordinates which bounds the slot shape
          	outlineCurves = slotShape.Curves;
            xMin          = min([outlineCurves.X0])*0.9;
            xMax          = max([outlineCurves.X0])*1.1;
            
            %% Draw a line through the center of the slot
            centerLine = Geometry1D.draw('Polyline','Points',[xMin, 0; xMax, 0]);
            
            %% Find where the center-line intersects the slot
            [~,~,s] = intersection([outlineCurves, centerLine]);
            s       = s(end);
            x       = centerLine.x(s{1});
            xMin    = min(x);
            xMax    = max(x);
            
            %% Restrict center-line to line between the conductorBoundaries
            xMin = max(xMin, conductorBoundaries(1));
            xMax = min(xMax, conductorBoundaries(2));

            %% Calculate y-coordinates which bound the slot shape
           	yMin = min([outlineCurves.Y0]);
            yMax = max([outlineCurves.Y0]);
            
            dy   = (yMax - yMin)*0.1;
            
            yMin = yMin-dy;
            yMax = yMax+dy;
                        
            xCenter = cell(1,0);
            yCenter = cell(1,0);
            if airgapLoc == AirgapLocations.Inside
                xPos = xMax-rConductor;
                sgn  = -1;
            else
                xPos = xMin+rConductor;
                sgn  = +1;
            end
            
            nPrev   = 0;
            nConductors = 0;
            while (xPos >= xMin+rConductor) && (xPos <= xMax - rConductor)
                nPos = inf;
                for i = -1:1
                    verticalLine = Geometry1D.draw('Polyline','Points',[xPos+i*rConductor, yMin; xPos+i*rConductor, yMax]);
                    [~,~,s] = intersection([outlineCurves, verticalLine]);
                    y    = verticalLine.y(s{end});
                    dy   = max(y)-min(y);
                    nPos = min(nPos,floor(dy/rConductor/2));
                end
                
                if (mod(nPos,2) == mod(nPrev,2)) && (nPrev ~= 0)
                    xPos = xPos + sgn*rConductor*(2-sqrt(3));                

                    nPos = inf;
                    for i = -1:1
                        verticalLine = Geometry1D.draw('Polyline','Points',[xPos+i*rConductor, yMin; xPos+i*rConductor, yMax]);
                        [~,~,s] = intersection([outlineCurves, verticalLine]);
                        y    = verticalLine.y(s{end});
                        dy   = max(y)-min(y);
                        nPos = min(nPos,floor(dy/rConductor/2));
                    end
                end
                
                if nPos > 0
                    if mod(nPos,2) == 1
                        yCenter{1,end+1} = 2*rConductor*(-floor(nPos/2):1:floor(nPos/2))';
                    else
                        yCenter{1,end+1} = 2*rConductor*(-(nPos/2):1:(nPos/2-1))'+rConductor;
                    end
                    xCenter{1,end+1} = xPos*ones(nPos,1);

                    xPos  = xPos + sgn*rConductor*sqrt(3);
                    nPrev = nPos;
                    nConductors = nConductors + nPos;
                else
                    nPrev = 0;
                    xPos  = xPos + sgn*rConductor;
                end
            end
            
            %% Make sure number of conductors is divisible by the number of turns
            nRemove = mod(nConductors, nTurns);
            nConductors = nConductors - nRemove;
            while nRemove > 0
                if length(xCenter{end}) <= nRemove
                    nRemove = nRemove - length(xCenter{end});
                    xCenter(end) = [];
                    yCenter(end) = [];
                else
                    nNew    = length(xCenter{end}) - nRemove;
                    nRemove = 0;
                    
                    if mod(nNew,2) == mod(length(xCenter{1,end-1}),2)
                    	xCenter{1,end} = ones(nNew,1)*(xCenter{1,end-1}(1)+sgn*rConductor*2);
                    else
                        xCenter{1,end} = ones(nNew,1)*(xCenter{1,end-1}(1)+sgn*rConductor*sqrt(3));
                    end
                    
                    if mod(nNew,2) == 1
                        yCenter{1,end} = 2*rConductor*(-floor(nNew/2):1:floor(nNew/2))';
                    else
                        yCenter{1,end} = 2*rConductor*(-(nNew/2):1:(nNew/2-1))'+rConductor;
                    end
                end
            end
            
            %% Draw conductors
        	conductorGeometry = Sector.empty(0,nConductors);
            k = 0;
            locationMatrix = cell(1,length(xCenter));
            for i = 1:length(xCenter)
                locationMatrix{1,i} = zeros(size(xCenter{i}));
                for j = 1:length(xCenter{i})
                    k = k + 1;
                    locationMatrix{i}(j) = k;
                    conductorGeometry(k) = Geometry2D.draw('Sector', 'Radius', [0 conductorDiameter / 2], 'Position', [xCenter{i}(j),yCenter{i}(j)], 'Angle', 2*pi,'PlotStyle',{'y'});
                end
         	end

            %% Assign Regions
            nonConductors = Region2D('Geometry', slotShape - conductorGeometry, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label , ' NC']);
            
            conductors = Region2D.empty(0,nConductors);
            for i = 1:nConductors
                conductors(i) = Region2D('Geometry', conductorGeometry(i), 'Material', this.ConductorMaterial, 'Dynamics', conductorDynamics, 'Name', [label , ' C', num2str(i)]);
            end
        end
        
        function [conductors, nonConductors, locationMatrix] = buildConcentratedWinding(this, slotShape, nTurns, nLayers, conductorDynamics, slotAngle, airgapLoc, label)
            %% Get parameters
            conductorDiameter   = this.ConductorDiameter;
            insulationThickness = this.InsulationThickness;
            rConductor          = conductorDiameter / 2 + insulationThickness;
            
            %% Calculate x-coordinates which bounds the slot shape
          	outlineCurves = slotShape.Curves;
            xMin          = min([outlineCurves.X0])*0.9;
            xMax          = max([outlineCurves.X0])*1.1;
            
            %% Draw a line through the center of the slot
            centerLine = Geometry1D.draw('Polyline','Points',[xMin, 0; xMax, 0]);
            
            %% Find where the center-line intersects the slot
            [~,~,s] = intersection([outlineCurves, centerLine]);
            s       = s(end);
            x       = centerLine.x(s{1});
            xMin    = min(x);
            xMax    = max(x);
            
            bbRad = 2*sum([outlineCurves.bbRadius]);
            
            if nLayers == 1
                slotShapeCopy = copy(slotShape);
            else %just use half of the slot
                pts = [xMax+bbRad, 0;xMin-bbRad, 0;xMin-bbRad, -bbRad; xMax+bbRad, -bbRad];
                slotLHPTrim = Geometry2D.draw('Polygon2D', 'Points', pts);
             	slotShapeCopy = slotShape * slotLHPTrim;
            end
            
         	rotate(slotShapeCopy, slotAngle/2, [0,0]);
            outlineCurves = slotShapeCopy.Curves;
            
            %% Try to find flat part of the slot
            N = ceil(2*sqrt(slotShape.area / (pi*rConductor^2)));
            x_test = linspace(xMin,xMax,N);
            y_test = zeros(1,N);
            
            for i = 1:N;
                verticalLine = Geometry1D.draw('Polyline','Points',[x_test(i),0; x_test(i),bbRad]);
                [~,~,s] = intersection([outlineCurves, verticalLine]);
                y = verticalLine.y(s{end});
                if ~isempty(y)
                	y_test(i) = min(y);
                else
                    y_test(i) = NaN;
                end
            end
            
            y_test(isnan(y_test)) = [];
            y_test = round(y_test/sqrt(eps))*sqrt(eps);
            [yMin,N] = mode(y_test);
            if N == 1 %if no flat edge is found, use the minimum that was found
                yMin = min(y_test);
            end
            
            %% Initialize iteration with base test points x0 and y0 to extrude in the y-direction
            yPos = yMin+rConductor;
            horzLine = Geometry1D.draw('Polyline','Points',[xMin-bbRad, yPos; xMax+bbRad, yPos]);
         	[~,~,s] = intersection([outlineCurves, horzLine]);
          	x = horzLine.x(s{end});
         	dx = max(x)-min(x);
        	nTest = floor(dx/rConductor/2);
            if airgapLoc == AirgapLocations.Inside
                xPos = min(x) + rConductor + rem(dx,2*rConductor) / 2;
                x0 = [xPos + 2*rConductor*(0:(nTest-1)), xPos + rConductor + 2*rConductor*(0:(nTest-2))];
            else
                xPos = max(x) - rConductor - rem(dx,2*rConductor) / 2;
                x0 = [xPos - 2*rConductor*(0:(nTest-1)), xPos - rConductor - 2*rConductor*(0:(nTest-2))];
            end
            y0 = [yPos*ones(1,nTest), (yPos+rConductor*sqrt(3))*ones(1,nTest-1)];
            nTest = 2*nTest - 1;
            
            xCenter = cell(1,nTest);
            yCenter = cell(1,nTest);
            nConductors = 0;
            for i = 1:nTest
                nPos = inf;
                for j = -1:1
                    vertLine = Geometry1D.draw('Polyline','Points',[x0(i)+j*rConductor, y0(i)-rConductor; x0(i)+j*rConductor, y0(i)+bbRad]);
                    [~,~,s] = intersection([outlineCurves, vertLine]);
                    y = vertLine.y(s{end});
                    dy = max(y)-y0(i)-rConductor;
                    if dy > 0
                    	nPos = min(nPos, ceil(dy/sqrt(3)/rConductor/2));
                    else
                        nPos = min(nPos, 0);
                    end
                end
                
                if nPos > 0
                 	xCenter{1,i} = x0(i)*ones(nPos,1);
                    yCenter{1,i} = y0(i) + 2*sqrt(3)*rConductor*(0:(nPos-1));
                    
                    nConductors = nConductors + nPos;
                end
            end
            
            %% Convert to rows
            xRows = cell(1,0);
            yRows = cell(1,0);
            isEmpty = false(1,nTest);
            for i = 1:nTest
                isEmpty(i) = isempty(xCenter{i});
            end
            
            while ~all(isEmpty)
                I = 1:(nTest+1)/2;
                
                if ~all(isEmpty(I))
                    xRows{1,end+1} = zeros(1,sum(~isEmpty(I)));
                    yRows{1,end+1} = zeros(1,sum(~isEmpty(I)));
                    j = 1;
                    for i = I
                        if ~isEmpty(i)
                            xRows{1,end}(j) = xCenter{1,i}(1);
                            yRows{1,end}(j) = yCenter{1,i}(1);

                            xCenter{1,i}(1) = [];
                            yCenter{1,i}(1) = [];

                            isEmpty(i) = isempty(xCenter{i});
                            j = j + 1;
                        end
                    end
                end
                
                I = ((nTest+1)/2+1):nTest;
                if ~all(isEmpty(I))
                    xRows{1,end+1} = zeros(1,sum(~isEmpty(I)));
                    yRows{1,end+1} = zeros(1,sum(~isEmpty(I)));
                    j = 1;
                    for i = I
                        if ~isEmpty(i)
                            xRows{1,end}(j) = xCenter{1,i}(1);
                            yRows{1,end}(j) = yCenter{1,i}(1);

                            xCenter{1,i}(1) = [];
                            yCenter{1,i}(1) = [];

                            isEmpty(i) = isempty(xCenter{i});
                            j = j + 1;
                        end
                    end
                end
            end
            xCenter = xRows;
            yCenter = yRows;
            
            %% Make sure number of conductors is divisible by the number of turns
            nRemove = mod(nConductors, nTurns);
            nConductors = nConductors - nRemove;
            
            if nRemove == nConductors
                error('Unable to fit more than %d conductors in the slot with the specified wire dimesions. This is less than the specified number of turns %d', nConductors, nTurns);
            end
            
            while nRemove > 0
                xCenter{end}(1) = [];
                yCenter{end}(1) = [];
                
                if isempty(xCenter{end})
                    xCenter(end) = [];
                    yCenter(end) = [];
                end
                
                nRemove  = nRemove - 1;
            end
            
            nRows = length(xCenter);
            
            rCenter = cell(1,nRows);
            if nLayers == 1
                aCenter = cell(2,nRows);
                for i = 1:length(xCenter)
                    rCenter{i}   = hypot(xCenter{i},yCenter{i});
                    aCenter{1,i} =  slotAngle/2-atan2(yCenter{i},xCenter{i});
                    aCenter{2,i} = -slotAngle/2+atan2(yCenter{i},xCenter{i});
                end
            else
                aCenter = cell(2,nRows);
                for i = 1:length(xCenter)
                    rCenter{i}   = hypot(xCenter{i},yCenter{i});
                    aCenter{1,i} =  atan2(yCenter{i},xCenter{i}) - slotAngle/2;
                    aCenter{2,i} = -atan2(yCenter{i},xCenter{i}) + slotAngle/2;
                end
            end
            
            %% Draw conductors
            conductors = cell(1,2);
            locationMatrix = cell(1,2);
            cGeom = cell(1,2);

            for l = 1:2
                cGeom{l} = Sector.empty(0,nConductors);
                locationMatrix{l} = cell(1,nRows);
                conductors{l} = Region2D.empty(0,nConductors);
                
                k = 0;
                for i = 1:nRows
                    locationMatrix{l}{i} = zeros(size(xCenter{i}));
                    ac = aCenter{l,i};
                    rc = rCenter{i};
                 	xc = rc.*cos(ac);
                   	yc = rc.*sin(ac);
                    
                    for j = 1:length(xCenter{i})
                        k = k + 1;
                        locationMatrix{l}{i}(j) = k;
                        cGeom{l}(k) = Geometry2D.draw('Sector', 'Radius', [0 conductorDiameter / 2], 'Position', [xc(j),yc(j)], 'Angle', 2*pi,'PlotStyle',{'y'});
                    end
                end
                
               	for i = 1:nConductors
                    conductors{l}(i) = Region2D('Geometry', cGeom{l}(i), 'Material', this.ConductorMaterial, 'Dynamics', conductorDynamics, 'Name', [label , ' C', num2str(i)]);
                end
            end
            
            if nLayers == 1
                ssCopy = copy(slotShape) - cGeom{1};
                nc1 = Region2D('Geometry', ssCopy, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label , ' NC1.1']);
                
                ssCopy = copy(slotShape) - cGeom{2};
                nc2 = Region2D('Geometry', ssCopy, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label , ' NC1.2']);
            else
                ssCopy = (copy(slotShape)*slotLHPTrim) - cGeom{1};
                nc1 = Region2D('Geometry', ssCopy, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label , ' NC1']);
                
                ssCopy = (copy(slotShape)-slotLHPTrim) - cGeom{2};
                nc2 = Region2D('Geometry', ssCopy, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label , ' NC1']);
            end
            nc1.Geometry.PlotStyle = {'w'};
            nc2.Geometry.PlotStyle = {'w'};
          	nonConductors = {nc1,nc2};
        end
    end
end