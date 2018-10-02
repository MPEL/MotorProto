function [model, stator, rotor] = make_CW_SMPM_Machine(model,f_r,nPoles,nTeeth,stackLength,turnsPerTooth,statorInnerRadius, statorBILength, statorBICutRadius, toothYokeLength, toothYokeWidth, toothFaceLength, toothFaceChamfer, toothGapWidth, slotFilletSize1, slotFilletSize2, slotPadding, slotPackingFactor,airgapLength, pmLength, pmEmbrace, pmFilletSize,rotorBILength, rotorBICutWidth, rotorBICutLength, statorIronMaterial, rotorIronMaterial, pmMaterial)
    assemblies = model.Assemblies;
    for i = 1:numel(assemblies)
        model.removeAssembly(assemblies(i).Name);
    end

    stator = model.newAssembly('CW Stator','Stator');
    rotor  = model.newAssembly('SMPM Rotor','SynchronousRotor');

    %% Derived Parameters
    statorBIRadius    = statorInnerRadius + statorBILength;
    statorOuterRadius = statorBIRadius + toothYokeLength + toothFaceLength;
    rotorInnerRadius  = statorOuterRadius + airgapLength;
    rotorOuterRadius  = rotorInnerRadius + pmLength + rotorBILength;

    %% Define Stator Geometry and Material Properties
    stator.ElectricalFrequency = f_r * nPoles / 2;
    stator.Length              = stackLength;
    stator.Poles               = nPoles;
    stator.Teeth               = nTeeth;
    stator.InnerRadius         = statorInnerRadius;
    stator.OuterRadius         = statorOuterRadius;
    stator.DefaultMaterial     = statorIronMaterial;
    stator.WindingType         = WindingTypes.Concentrated;
    stator.CouplingType        = CouplingTypes.Static;
    stator.Slot.Layers         = 2;
    stator.Slot.Turns          = turnsPerTooth;
    stator.Slot.AirgapLocation = AirgapLocations.Outside;
    
    %% Bus-Bar Style Conductors
    stator.Slot.ConductorType           = ConductorTypes.Homogenized;
    stator.Slot.Conductor.PackingFactor = slotPackingFactor;

    stator.Sources.ElectricalFrequency = f_r * nPoles / 2;
    
    %% Define slot geometry
    %% Slot Polygon
    rSlot      = [statorInnerRadius+statorBILength,statorInnerRadius+statorBILength+toothYokeLength, statorOuterRadius];
    aSlot      = 2 * pi / nTeeth;
    slotSector = Geometry2D.draw('Sector', 'Radius', [statorInnerRadius,statorOuterRadius] ,'Angle',aSlot, 'Rotation',-aSlot / 2);

    xTooth1 = rSlot(1) * cos(aSlot * toothYokeWidth / 2);
    yTooth1 = rSlot(1) * sin(aSlot * toothYokeWidth / 2);

    xTooth2 = rSlot(2) * cos(aSlot * toothYokeWidth / 2);
    yTooth2 = yTooth1;
    
    m       = tan(aSlot/2);
    b       = - toothGapWidth * cos((pi-aSlot)/2)/2;
    
    xTooth3 = xTooth2;
    yTooth3 = m*xTooth3+b;
    
    xTooth4 = rotorInnerRadius;
    yTooth4 = m*xTooth4+b;
    
    pSlot   = [xTooth1,yTooth1;
                xTooth2,yTooth2;
                xTooth3,yTooth3;
                xTooth4,yTooth4;
                xTooth4,-yTooth4;
                xTooth3,-yTooth3;
                xTooth2,-yTooth2;
                xTooth1,-yTooth1];
            
    %% Conductor Polygon
    pCond = [xTooth1 + slotPadding,  yTooth1 + slotPadding;
             xTooth2 - slotPadding,  yTooth2 + slotPadding;
             xTooth2 - slotPadding, -yTooth2 - slotPadding;
             xTooth1 + slotPadding, -yTooth1 - slotPadding];

 	%% Rotate Geometry to Center the Slot
    rSlot = hypot(pSlot(:,1),pSlot(:,2));
    aSlot = atan2(pSlot(:,2),pSlot(:,1));
    aSlot = aSlot + pi/nTeeth * [-1;-1;-1;-1;1;1;1;1];
    pSlot = [rSlot .* cos(aSlot), rSlot .* sin(aSlot)];

    %% Calculate Chamfer
    m1 = (pSlot(3,2)-pSlot(2,2))/(pSlot(3,1)-pSlot(2,1));
    b1 = (pSlot(3,2)+pSlot(2,2)-m1*(pSlot(3,1)+pSlot(2,1)))/2;
    
    y32 = pSlot(4,2);
    x32 = pSlot(3,1) + toothFaceChamfer*(pSlot(4,1)-pSlot(3,1));
    
    m2 = 1;
    b2 = y32-m2*x32;
    
    x31 = -(b2-b1)/(m2-m1);
    y31 = ((m2+m1)*x31+b2+b1)/2;
    
    pSlot = [pSlot(1:2,:);
             x31,y31;
             x32,y32;
             pSlot(4:5,:);
             x32,-y32;
             x31,-y31;
             pSlot(7:8,:)];
         
    %% Calculate Slot Fillet
    slotFillet    = (statorBIRadius * sin(pi/nTeeth) - yTooth1) / (1-sin(pi/nTeeth)) * slotFilletSize1;
    ySlotFillet   = [yTooth1 + slotFillet,-yTooth1-slotFillet];
    angSlotFillet = abs(atan2(sin(2 * pi / nTeeth*toothYokeWidth/2)-sin(2*pi/nTeeth-2 * pi / nTeeth*toothYokeWidth/2),cos(2 * pi / nTeeth*toothYokeWidth/2)-cos(2*pi/nTeeth-2 * pi / nTeeth*toothYokeWidth/2)));
    yInt          = yTooth1 + slotFillet * (1 - cos(angSlotFillet));
    xInt          = (-yTooth1+yInt)/tan(-angSlotFillet)+xTooth1;
    xSlotFillet   = [xInt,xInt]+slotFillet/ sqrt(1+tan(pi/2-angSlotFillet)^2);
    rotSlotFillet = [-pi/2-angSlotFillet,pi/2];

    slotFilletSize2 = slotFilletSize2*(hypot(diff(pSlot(2:3,2)), diff(pSlot(2:3,1))));
    
    xSlotFillet   = [xSlotFillet, xTooth2-slotFilletSize2, xTooth2-slotFilletSize2];
    ySlotFillet   = [ySlotFillet, yTooth2+slotFilletSize2, -yTooth2-slotFilletSize2];
    angSlotFillet = [angSlotFillet,  pi/2];
    rotSlotFillet = [rotSlotFillet, -pi/2, 0];
    
%     %% Rotate Geometry to Center the Slot
%     rSlot = hypot(pSlot(:,1),pSlot(:,2));
%     aSlot = atan2(pSlot(:,2),pSlot(:,1));
%     aSlot = aSlot + pi/nTeeth * [-1;-1;-1;-1;1;1;1;1];
%     pSlot = [rSlot .* cos(aSlot), rSlot .* sin(aSlot)];
% 
%     %% Calculate Chamfer
%     m1 = (pSlot(3,2)-pSlot(2,2))/(pSlot(3,1)-pSlot(2,1));
%     b1 = (pSlot(3,2)+pSlot(2,2)-m1*(pSlot(3,1)+pSlot(2,1)))/2;
%     
%     y32 = pSlot(4,2);
%     x32 = pSlot(3,1) + toothFaceChamfer*(pSlot(4,1)-pSlot(3,1));
%     
%     m2 = 1;
%     b2 = y32-m2*x32;
%     
%     x31 = -(b2-b1)/(m2-m1);
%     y31 = ((m2+m1)*x31+b2+b1)/2;
%     
%     pSlot = [pSlot(1:2,:);
%              x31,y31;
%              x32,y32;
%              pSlot(4:5,:);
%              x32,-y32;
%              x31,-y31;
%              pSlot(7:8,:)];
         
    %% Rotate Conductor and Fillets
         
    rCond = hypot(pCond(:,1),pCond(:,2));
    aCond = atan2(pCond(:,2),pCond(:,1));
    aCond = aCond + pi/nTeeth * [-1;-1;1;1];
    pCond = [rCond .* cos(aCond), rCond .* sin(aCond)];

    rotSlotFillet = rotSlotFillet + [-pi/nTeeth,pi/nTeeth,-pi/nTeeth,pi/nTeeth];
    radSlotFillet = hypot(xSlotFillet,ySlotFillet);
    aSlotFillet   = atan2(ySlotFillet,xSlotFillet) + [-pi/nTeeth,pi/nTeeth,-pi/nTeeth,pi/nTeeth];
    xSlotFillet   = radSlotFillet .* cos(aSlotFillet);
    ySlotFillet   = radSlotFillet .* sin(aSlotFillet);

    %% Calculate Conductor Fillet
    m           = (pCond(3,2) - pCond(4,2)) / (pCond(3,1) - pCond(4,1));
    b           = pCond(4,2) - m * pCond(4,1);
    xl          = pCond(4,1);
    f           = (1/m+m)*sqrt(1/(1+m^2)) - 1;
    condFillet  = (xl + b/m) / f * slotFilletSize1;

    xCondFillet = [pCond(1,1),pCond(1,1)] + condFillet;
    mLine       = -m;
    mRecp       = -1 / mLine;
    dx          = condFillet / sqrt(1+mRecp^2);
    dy          = mRecp * dx;
    bLine       = pCond(1,2) - mLine * pCond(1,1);

    yCondFillet    = dy + mLine * (xCondFillet(1) - dx) + bLine;
    angCondFillet  = atan2(dy,dx);
    rotCondFillet  = [-pi,pi-angCondFillet];
    yCondFillet(2) = -yCondFillet(1);
    
    condFilletSize2 = slotFilletSize2 - slotPadding;
    xCondFillet = [xCondFillet,pCond(2,1)-condFilletSize2,pCond(2,1)-condFilletSize2];
    
    m = diff(pCond(1:2,2))/diff(pCond(1:2,1));
    b = pCond(2,2)-m*pCond(2,1);
    b = b + hypot(1,m)*condFilletSize2;
    
    yCondFillet = [yCondFillet,m*xCondFillet(3)+b];
    yCondFillet = [yCondFillet,-yCondFillet(3)];
    
    angCondFillet = [angCondFillet, pi-angCondFillet];
    rotCondFillet = [rotCondFillet, -pi+angCondFillet(1),0];
    
    %% Seperate Face
    %pSlotBody = pSlot([1 2 9 10],:);
    %pSlotFace = pSlot([2 3 4 5 6 7 8 9],:);

    slotBody  = Geometry2D.draw('Polygon2D','Points',pSlot);
    %slotBody = Geometry2D.draw('Polygon2D','Points',pSlotBody);
    slotCond = Geometry2D.draw('Polygon2D','Points',pCond,'PlotStyle',{'y'});
    %slotFace = Geometry2D.draw('Polygon2D','Points',pSlotFace);

    slotFillet1 = Geometry2D.draw('Sector','Radius',[slotFillet,2*slotFillet],'Angle',angSlotFillet(1),'Rotation',rotSlotFillet(1),'Position',[xSlotFillet(1),ySlotFillet(1)]);
    slotFillet2 = Geometry2D.draw('Sector','Radius',[slotFillet,2*slotFillet],'Angle',angSlotFillet(1),'Rotation',rotSlotFillet(2),'Position',[xSlotFillet(2),ySlotFillet(2)]);
    slotFillet3 = Geometry2D.draw('Sector','Radius',[slotFilletSize2,2*slotFilletSize2],'Angle',angSlotFillet(2),'Rotation',rotSlotFillet(3),'Position',[xSlotFillet(3),ySlotFillet(3)]);
    slotFillet4 = Geometry2D.draw('Sector','Radius',[slotFilletSize2,2*slotFilletSize2],'Angle',angSlotFillet(2),'Rotation',rotSlotFillet(4),'Position',[xSlotFillet(4),ySlotFillet(4)]);
        
    condFillet1 = Geometry2D.draw('Sector','Radius', [condFillet,2*condFillet],'Angle',angCondFillet(1),'Rotation',rotCondFillet(1),'Position',[xCondFillet(1),yCondFillet(1)]);
    condFillet2 = Geometry2D.draw('Sector','Radius',[condFillet,2*condFillet],'Angle',angCondFillet(1),'Rotation',rotCondFillet(2),'Position',[xCondFillet(2),yCondFillet(2)]);
  	
    if condFilletSize2 > 0
        condFillet3 = Geometry2D.draw('Sector','Radius', [condFilletSize2,2*condFilletSize2],'Angle',angCondFillet(2),'Rotation',rotCondFillet(3),'Position',[xCondFillet(3),yCondFillet(3)]);
        condFillet4 = Geometry2D.draw('Sector','Radius',[condFilletSize2,2*condFilletSize2],'Angle',angCondFillet(2),'Rotation',rotCondFillet(4),'Position',[xCondFillet(4),yCondFillet(4)]);
    end
    
%     slotCond    = ((slotCond - condFillet1) - condFillet2);
    slotCond    = slotCond - condFillet1;
    slotCond    = slotCond - condFillet2;
    
    if condFilletSize2 > 0
        slotCond    = slotCond - condFillet3;
        slotCond    = slotCond - condFillet4;
    end
    
    slotBody    = slotBody - slotFillet1;
    slotBody    = slotBody - slotFillet2;
    slotBody    = slotBody - slotFillet3;
    slotBody    = slotBody - slotFillet4;
    slotBody    = slotBody - slotCond;
    slotBody    = slotBody * slotSector;
    
    %slotBody    = (((slotBody - slotFillet1) - slotFillet2) - slotCond) * slotSector;
    %slotFace    = slotFace * slotSector;
    %slotAir     = slotBody + slotFace;
    
    stator.Slot.Shape        = slotCond;
    stator.ConductorMaterial = Copper;

    stator.addRegion('slotPadding', slotBody, Air, DynamicsTypes.Static);
    
    %% Cut Stator Backiron
    scut1 = Geometry2D.draw('Sector','Radius',[0,statorBICutRadius],'Angle',2*pi,'Rotation',0,'Position',statorInnerRadius*[cos(pi/nTeeth),sin(pi/nTeeth)]);
    scut2 = Geometry2D.draw('Sector','Radius',[0,statorBICutRadius],'Angle',2*pi,'Rotation',0,'Position',statorInnerRadius*[cos(-pi/nTeeth),sin(-pi/nTeeth)]);
    
    scut1 = scut1 * slotSector;
    scut2 = scut2 * slotSector;
    
    stator.addRegion('scut1', scut1, Air, DynamicsTypes.Static);
    stator.addRegion('scut2', scut2, Air, DynamicsTypes.Static);
    
    %% Set Rotor Parameters
    rotor.Poles               = nPoles;
    rotor.Length              = stackLength;
    rotor.ElectricalFrequency = f_r * nPoles / 2;
    rotor.InnerRadius         = rotorInnerRadius - airgapLength / 2;
    rotor.OuterRadius         = rotorOuterRadius;
    rotor.DefaultMaterial     = rotorIronMaterial;
    rotor.OperatingMode       = OperatingModes.Synchronous;
    rotor.InitialAngle        = 0;

    %% Create Rotor Permanent Magnet
    pmWidth  = 2*pi/nPoles * pmEmbrace;

    %% Calculate Magnet Fillet
  	permanentMagnet = Geometry2D.draw('Sector', 'Radius', [rotorInnerRadius, rotorInnerRadius + pmLength], 'Angle', pmWidth, 'Rotation', - pmWidth / 2,'PlotStyle',{'m'});
    
    if rotorInnerRadius * sin(pmWidth / 2) > pmLength
        pmFillet = pmLength * pmFilletSize / 2;
    else
        pmFillet = rotorInnerRadius * sin(pmWidth / 2) * pmFilletSize / 2;
    end

    m =   tan(pmWidth / 2);
    b = - pmFillet * hypot(1,m);
    r = rotorInnerRadius+pmFillet;
    x = (-2*m*b+sqrt(4*m^2*b^2-4*(1+m^2)*(b^2-r^2)))/(2*(1+m^2));
    y = m*x+b;
    c = Geometry2D.draw('Sector','Radius',[0 pmFillet],'Angle',2*pi,'Rotation',0,'Position',[x,y]);
    [x1,y1] = intersection([permanentMagnet.Curves(3),c.Curves(1)]);
    [x2,y2] = intersection([permanentMagnet.Curves(4),c.Curves(1)]);
    a1  = atan2(y1-y,x1-x);
    a2  = atan2(y2-y,x2-x) + 2 * pi;
    rotFillet = [a1,0];
    angFillet = [a2-a1,0];
    c = Geometry2D.draw('Sector','Radius',[0 pmFillet],'Angle',2*pi,'Rotation',0,'Position',[x,-y]);
    [x1,y1] = intersection([permanentMagnet.Curves(4),c.Curves(1)]);
    [x2,y2] = intersection([permanentMagnet.Curves(1),c.Curves(1)]);
    a1  = atan2(y1+y,x1-x)-2*pi;
    a2  = atan2(y2+y,x2-x);
    rotFillet(2) = a2-(a2-a1);
    angFillet(2) = a2-a1;
    xFillet = [x x];
    yFillet = [y -y];

    magnetFillet1   = Geometry2D.draw('Sector', 'Radius', [pmFillet, 2*pmFillet], 'Angle', angFillet(1), 'Rotation', rotFillet(1), 'Position',[xFillet(1),yFillet(1)]);            
    magnetFillet2   = Geometry2D.draw('Sector', 'Radius', [pmFillet, 2*pmFillet], 'Angle', angFillet(2), 'Rotation', rotFillet(2), 'Position',[xFillet(2),yFillet(2)]);
    magnetLocation  = Geometry2D.draw('Sector', 'Radius', [rotorInnerRadius - airgapLength / 2, rotorInnerRadius + pmLength], 'Angle', 2*pi / nPoles, 'Rotation', -pi/nPoles);
    permanentMagnet = (permanentMagnet - magnetFillet1) - magnetFillet2;
    magnetGap       = magnetLocation - permanentMagnet;

    rotor.addRegion('pm', permanentMagnet,  pmMaterial, DynamicsTypes.Static);
    rotor.addRegion('trimUHP', magnetGap, Air, DynamicsTypes.Static);
    
    %%Cut Rotor Backiron
	poleSector = Geometry2D.draw('Sector', 'Radius', [rotorInnerRadius,rotorOuterRadius] ,'Angle',2*pi/nPoles, 'Rotation',-pi/nPoles);

    %     rcut       = Geometry2D.draw('Sector', 'Radius', [0,rotorBICutRadius],'Angle',2*pi,'Rotation',0,'Position', [rotorOuterRadius,0]);
%     rcut       = rcut * poleSector;

    x1 = rotorOuterRadius * cos(pi/nPoles * rotorBICutWidth);
    y1 = rotorOuterRadius * sin(pi/nPoles * rotorBICutWidth);
    x2 = x1 - rotorBICutLength;
    y2 = y1 - x1 + x2;
     m = 1;
     b = y2-x2;
	x1 = rotorOuterRadius * 1.01;
	y1 = m*x1 + b;
    p  = [  x1 y1;
            x2 y2;
            x2 -y2;
            x1 -y1];
        
    rcut = Geometry2D.draw('Polygon2D','Points',p);
    rcut = rcut * poleSector;
    rotor.addRegion('rcut', rcut, Air, DynamicsTypes.Static);
end