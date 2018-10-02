classdef HomogenizedConductor < Wire
    %HomogenizedConductor.m A class representing a bulk averaged conductor model
    %   HomogenizedConductor objects
    %
    % HomogenizedConductor properties:
    %   PackingFactor       - Ratio of conducting material to total material
    %   HomogenizedMaterial - Output of the material homogenization process
    %
  	% HomogenizedConductor methods:
    %   build - Divides an input region into a number of conductors
    %
    % HomogenizedConductor inherits properties from Wire.
    %
    % See the help for Wire for more information.
    %
    % See also MotorProto, Model, Assembly, Wire
    
%{
properties:
 	%PackingFactor - Ratio of conducting material to total material
    %   
    %
    %
    % See also HomogenizedConductor
    PackingFactor;
    
 	%HomogenizedMaterial - Output of the material homogenization process
    %
    % See also HomogenizedConductor, HomogenizedMaterialProperty
    HomogenizedMaterial;
%}
    
    properties
        PackingFactor = 1;
    end
    
    properties (Dependent)
        HomogenizedMaterial
    end
    
    methods
        function value = get.HomogenizedMaterial(this)
            m     = [this.ConductorMaterial, this.InsulatorMaterial];
            p     = [this.PackingFactor, 1 - this.PackingFactor];
            value = HeterogeneousMaterial('BaseProperties', m, 'Percentage', p);
        end
        
        function [conductors, nonConductors, locationMatrix] = build(this, slotShape, conductorBoundaries, nTurns, nLayers, conductorDynamics, couplingType, windingType, ~, ~, label)
            switch windingType
                case WindingTypes.Distributed
                    [conductors, nonConductors, locationMatrix] = buildDistributedWinding(this, slotShape, conductorBoundaries, nTurns, conductorDynamics, couplingType, label);
                case WindingTypes.Concentrated
                    [conductors, nonConductors, locationMatrix] = buildConcentratedWinding(this, slotShape, conductorBoundaries, nLayers, conductorDynamics, label);
                otherwise
                    error('Unknown winding type %s', char(windingType));
            end
        end
        
        function [conductors, nonConductors, locationMatrix] = buildDistributedWinding(this, slotShape, conductorBoundaries, nTurns, conductorDynamics, couplingType, label)
            if couplingType == CouplingTypes.Static
                nTurns = 1;
            end
            
            %% Get Bounding Box of slotShape
            curves  = slotShape.Curves;
            nCurves = numel(curves);
            xBounds = [inf 0];
            yBounds = [inf -inf];
            for i = 1:nCurves;
                xBounds(1)  = min(xBounds(1), curves(i).bbCenter(1) - curves(i).bbRadius);
                xBounds(2)  = max(xBounds(2), curves(i).bbCenter(1) + curves(i).bbRadius);
                
                yBounds(1)  = min(yBounds(1), curves(i).bbCenter(2) - curves(i).bbRadius);
                yBounds(2)  = max(yBounds(2), curves(i).bbCenter(2) + curves(i).bbRadius);
            end
            
            xMin = 0;
            yMin = yBounds(1);
            yMax = yBounds(2);
            
            %% Divide into Conducting/Nonconducting Regions
            if ~isinf(conductorBoundaries(1)) && ~isinf(conductorBoundaries(2))
                %% Two Boundaries
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [conductorBoundaries(1), yMin;
                                                                                  conductorBoundaries(2), yMin;
                                                                                  conductorBoundaries(2), yMax;
                                                                                  conductorBoundaries(1), yMax]);
                conductingRegion        = slotShape * testShape;
                
                nonConductors           = Region2D.empty(0,2);
                
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [conductorBoundaries(1), yMin;
                                                                                  conductorBoundaries(1), yMax;
                                                                                  xBounds(1)            , yMax;
                                                                                  xBounds(1)            , yMin]);
                                                                              
                nonConductors(1)        = Region2D('Geometry', slotShape * testShape, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static,...
                                                   'Name', [label, ' NC1']);
                
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [conductorBoundaries(2), yMin;
                                                                                  xBounds(2)            , yMin;
                                                                                  xBounds(2)            , yMax;
                                                                                  conductorBoundaries(2), yMax]);
                                                                              
                nonConductors(2)        = Region2D('Geometry', slotShape * testShape, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static,...
                                                   'Name', [label, ' NC2']);
                
                xBounds(1)              = conductorBoundaries(1) * (1 - sqrt(eps));
                xBounds(2)              = conductorBoundaries(2) * (1 + sqrt(eps));
            elseif ~isinf(conductorBoundaries(1))
                %% Inner Boundary Only
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [conductorBoundaries(1), yMin;
                                                                                  xBounds(2), yMin;
                                                                                  xBounds(2), yMax;
                                                                                  conductorBoundaries(1), yMax]);
                conductingRegion        = slotShape * testShape;
                
                testShape               = Geometry2D.draw('Polygon2D','Points', [conductorBoundaries(1), yMin;
                                                                                 conductorBoundaries(1), yMax;
                                                                                 xBounds(1)            , yMax;
                                                                                 xBounds(1)            , yMin]);
                nonConductors           = Region2D('Geometry', slotShape * testShape, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static,...
                                                   'Name', [label, ' NC']);
                
                xBounds(1)              = conductorBoundaries(1) * (1 - sqrt(eps));
            elseif ~isinf(conductorBoundaries(2))
                %% Outer Boundary Only
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [xBounds(1), yMin;
                                                                                  conductorBoundaries(2), yMin;
                                                                                  conductorBoundaries(2), yMax;
                                                                                  xBounds(1), yMax]);
                conductingRegion        = slotShape * testShape;
                
                testShape               = Geometry2D.draw('Polygon2D', 'Points', [conductorBoundaries(2), yMin;
                                                                                  xBounds(2)            , yMin;
                                                                                  xBounds(2)            , yMax;
                                                                                  conductorBoundaries(2), yMax]);

                nonConductors           = Region2D('Geometry', slotShape * testShape, 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static,...
                                                   'Name', [label, ' NC']);
                
                xBounds(2)              = conductorBoundaries(2) * (1 + sqrt(eps));
            else
                %% No Boundaries
                conductingRegion = slotShape;
                nonConductors    = Region2D.empty(0,1);
            end
            
            %% Get Good Initial Polynomial Area Estimate
           	A  = area(conductingRegion);
            dA = A / nTurns;
            x  = [(xBounds(1)*3+xBounds(2))/4 (xBounds(1)+xBounds(2))/2 (xBounds(1)+xBounds(2)*3)/4];
            a  = zeros(1,3);
            for i = 1:3
                testRegion = Geometry2D.draw('Polygon2D','Points', [xMin, yMin;x(i), yMin; x(i), yMax; xMin, yMax]);
                testTurn   = slotShape * testRegion;
                a(i)       = area(testTurn);
            end
            
            %% Divide Conducting Region in Massive Turns
            conductorGeometry = Polygon2D.empty(0, nTurns);
            for i = 1:nTurns
                aNew = 0;
                while abs(aNew - dA) > dA * sqrt(eps)
                    p      = polyfit(x,a,2);
                    p(end) = p(end) - i*dA;
                    
                    if i == nTurns
                        xMax = max(xBounds) * 2;
                    else
                        xMax = real(roots(p));

                        isInRange = (xMax > xBounds(1)) & (xMax < xBounds(2));
                        if any(isInRange)
                            xMax = max(xMax(isInRange));
                        else
                            xMax = xBounds(2);
                        end
                    end
                    
                    testRegion           = Geometry2D.draw('Polygon2D', 'Points', [xMin, yMin; xMax, yMin; xMax, yMax; xMin, yMax]);
                    conductorGeometry(i) = conductingRegion * testRegion;
                    aNew                 = area(conductorGeometry(i));
                    
                    if abs(aNew - dA) > dA * sqrt(eps)
                        x = [x(2:3) xMax];
                        a = [a(2:3) (i-1)*dA + aNew];
                    end
                end
                xMin = xMax;
            end
            
            conductors = Region2D.empty(0, nTurns);
            for i = 1:nTurns
                conductors(i) = Region2D('Geometry', conductorGeometry(i), 'Material', this.HomogenizedMaterial, 'Dynamics', conductorDynamics, 'Name', [label, ' C', num2str(i)]);
            end
            
            locationMatrix = num2cell(1:nTurns);
        end
        
        function [conductors, nonConductors, locationMatrix] = buildConcentratedWinding(this, slotShape, cBdry, nLayers, conductorDynamics, label)
            assert(conductorDynamics == DynamicsTypes.Static, 'Homogenized conductors for concentrated windings must have DynamicsTypes.Static');
            
            %% Get Bounding Box of slotShape
            curve  = slotShape.Curves;
            nCurve = numel(curve);
            xBnds = [inf 0];
            yBnds = [inf -inf];
            for i = 1:nCurve
                xBnds(1)  = min(xBnds(1), curve(i).bbCenter(1) - curve(i).bbRadius);
                xBnds(2)  = max(xBnds(2), curve(i).bbCenter(1) + curve(i).bbRadius);
                
                yBnds(1)  = min(yBnds(1), curve(i).bbCenter(2) - curve(i).bbRadius);
                yBnds(2)  = max(yBnds(2), curve(i).bbCenter(2) + curve(i).bbRadius);
            end
            
            yMin = yBnds(1);
            yMax = yBnds(2);
            
            %% Divide into Conducting/Nonconducting Regions
            if ~isinf(cBdry(1)) && ~isinf(cBdry(2))
                %% Two Boundaries
                pts = [cBdry(1), yMin; cBdry(2), yMin; cBdry(2), yMax; cBdry(1), yMax];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                cGeom  = slotShape * trim;
                
                ncGeom = Geometry2D.empty(1,0);
                
                pts = [cBdry(1), yMin; cBdry(1), yMax; xBnds(1), yMax; xBnds(1), yMin];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                ncGeom(1) = slotShape * trim;
                
                pts = [cBdry(2), yMin; xBnds(2), yMin; xBnds(2), yMax; cBdry(2), yMax];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                ncGeom(2) = slotShape * trim;
                
                xBnds(1) = cBdry(1) * (1 - sqrt(eps));
                xBnds(2) = cBdry(2) * (1 + sqrt(eps));
            elseif ~isinf(cBdry(1))
                %% Inner Boundary Only
                pts = [cBdry(1), yMin;xBnds(2), yMin;xBnds(2), yMax; cBdry(1), yMax];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                cGeom = slotShape * trim;
                
                pts = [cBdry(1), yMin;cBdry(1), yMax;xBnds(1), yMax;xBnds(1), yMin];
                trim = Geometry2D.draw('Polygon2D','Points', pts);
                ncGeom = slotShape * trim;
                
                xBnds(1) = cBdry(1) * (1 - sqrt(eps));
            elseif ~isinf(cBdry(2))
                %% Outer Boundary Only
                pts = [xBnds(1), yMin; cBdry(2), yMin; cBdry(2), yMax; xBnds(1), yMax];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                cGeom = slotShape * trim;
                
                pts = [cBdry(2), yMin;xBnds(2), yMin;xBnds(2), yMax;cBdry(2), yMax];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                ncGeom = slotShape * trim;
                
                xBnds(2) = cBdry(2) * (1 + sqrt(eps));
            else
                %% No Boundaries
                cGeom  = slotShape;
                ncGeom = slotShape.empty(0,1);
            end
            
            if nLayers == 1
                conductors    = cell(1,2);
                nonConductors = {Region2D.empty(1,0),Region2D.empty(1,0)};
                
             	conductors{1} = Region2D('Geometry', cGeom, 'Material', this.HomogenizedMaterial, 'Dynamics', conductorDynamics, 'Name', [label, ' C', num2str(1)]);
                for i = 1:numel(ncGeom)
                    nonConductors{1}(i) = ...
                        Region2D('Geometry', ncGeom(i), 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label, ' NC1.', num2str(i)]);
                end
                
                cGeom  = copy(cGeom);
                ncGeom = copy(ncGeom);
                
                conductors{2} = Region2D('Geometry', cGeom, 'Material', this.HomogenizedMaterial, 'Dynamics', conductorDynamics, 'Name', [label, ' C', num2str(2)]);
                for i = 1:numel(ncGeom)
                    nonConductors{2}(i) = ...
                        Region2D('Geometry', ncGeom(i), 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label, ' NC2.', num2str(i)]);
                end
            else
                pts = [xBnds(1), 0; xBnds(2), 0; xBnds(2), yBnds(2); xBnds(1), yBnds(2)];
                trim = Geometry2D.draw('Polygon2D', 'Points', pts);
                cGeom = [cGeom - trim, cGeom * trim];

                conductors = {Region2D.empty(1,0),Region2D.empty(1,0)};
                for i = 1:2
                    conductors{i} = Region2D('Geometry', cGeom(i), 'Material', this.HomogenizedMaterial, 'Dynamics', conductorDynamics,'Name', [label, ' C', num2str(i)]);
                end
                
                nonConductors = {Region2D.empty(1,0),Region2D.empty(1,0)};
                for j = 1:2
                    for i = 1:numel(ncGeom)
                        nonConductors{j}(i) = ...
                            Region2D('Geometry', ncGeom(i), 'Material', this.InsulatorMaterial, 'Dynamics', DynamicsTypes.Static, 'Name', [label, ' NC', num2str(j),'.',num2str(i)]);
                    end
                end
            end
            locationMatrix = {{1},{1}};
        end
    end
end