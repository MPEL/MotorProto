function [CG, NCG] = slotTemplate(NS, RI, RO, NW, NL, SW, SL, SA, CO, varargin)
%slotTemplate Creates geometry for a stator slot
%   [CG, NCG] = slotTemplate(NS, RI, RO, NW, NL, SW, SL, SA, CO, 'parameter1', value1, ...)
%   constructs the geometry of a stator slot using Geometry2D objects based on a
%   predefined template. CG is the region where conductors can be allocated.
%   NCG is the region where conductors will not be allocated. 
%
%   Arguments:
%   NS - Specifies the number of slots in the stator. 
%   RI - Specifies the inner radius of the stator.
%   RO - Specifies the outer radius of the stator.
%   NW - Defines the width of the notch placed between the face of the stator teeth
%        as a percentage of 2*pi/NS. 
%   NL - Defines the length of the notch between the face of the stator teeth as a
%        percentage of (RO - RI).
%   SW - Defines the width of the main part ofthe slot closest to the face of the
%        teeth as a percentage of 2*pi/NS.
%   SL - Defines the length of the main part of the slot as a function of (RO - RI).
%   SA - Specifies the angle of the sides of the slot as a percentage of pi/NS.
%   CO - Specifies the conductor/nonconductor region boundary. When CO > 0, CG
%        includes the portion of the slot such that x > RI * (1 - CO) + RO * CO.
%        When CO = 0, CG is the entire slot and NCG is an empty Geometry2D
%        object. If CO = 'auto', the location of the split is automatically
%        determined based on the notch width at the parameter/value pairs
%        determining the inner slot shape (see below).
%
%   Details:
%   Using these parameters, the length of the stator tooth is approximately
%   (RO - RI) * (NL + SL) measured along the radial direction. Similarly, the
%   maximum radial length of the backiron of the stator is given by 
%   (RO - RI) * (1 - NL - SL).
%   
%   Two specific choices of SA are of interest. SA = 0 results in a slot with
%   straight sides (constant cross sectional area as a function of radius). SA = 1
%   results in a tooth with straight sides. That is, the tooth implicitly defined
%   by the slot shape has a constant cross sectional area as a function of radius. 
%
%   In addition to the 9 required arguments, there are 4 optional parameter/value
%   pairs which allow you to further customize the slot shape.
%
%   Parameter/Value Pairs:
%       'InnerSlotShape'
%           {'rounded'} | 'straight'
%           Controls the shape of the slot near the tooth face. Rounded
%           (default) automatically rounds the inner edges of the slot. Straight
%           will draw straight edges connecting the tooth face to the tooth
%           body.
%
%       'InnerSlotLength'
%           {0} < p < NS
%           When InnerSlotShape = 'straight', controls the transition from the
%           tooth face to the tooth body.
%
%       'OuterSlotShape'
%           {'rounded'} | 'straight'
%           Controls the shape of the slot near the backiron. Rounded
%           (default) automatically rounds the outer face of the slot. Straight
%           will draw a straight edges to close the tooth near the backiron.
%
%       'OuterSlotLength'
%           {0} < p < NS
%           When OuterSlotShape = 'straight', controls the transition from the
%           body of the tooth to the backiron.
%
%       'AirgapLocation'
%           {'inside'} | 'outside'
%           Specifies the location of the airgap relative to the annulus
%           containing the slot. This parameter controls the placement of
%           the slot notch NCG.
%
%       Note: When both InnerSlotShape and OuterSlotShape are selected as
%       'straight', it is required that InnerSlotLength + OuterSlotLength < NS.
%
%   Example:
%      [cg,ncg] = slotTemplate(72, 0.08513, 0.12, 0.05, 0.01, 0.5, 0.5, 1, 'auto',...
%                               'OuterSlotShape', 'Rounded',...
%                               'InnerSlotShape', 'Straight', 'InnerSlotLength', 0.02);
%      cg.PlotStyle = {'y'};
%      ncg.PlotStyle = {'w'};
%      figure;hold on;
%      plot(cg);
%      plot(ncg);
%
%   See also Geometry2D, Slot, Stator

    %% Parse Inputs
  	ip = inputParser;
    
    ip.addRequired('slots', @(x)(x>0));
    ip.addRequired('innerRadius', @(x)(x>0));
    ip.addRequired('outerRadius', @(x)(x>0));
    ip.addRequired('notchWidth', @(x)(x>0 && x < 1));
    ip.addRequired('notchLength', @(x)(x>0 && x < 1));
    ip.addRequired('slotWidth', @(x)(x>0 && x < 1));
    ip.addRequired('slotLength', @(x)(x>0 && x < 1));
    ip.addRequired('slotAngle', @(x)(x>=0 && x <= 1));
    ip.addRequired('conductorOffset', @(x)(strcmpi(x,'auto') || (x>=0 && x < 1)));
    
    ip.addParamValue('InnerSlotShape', 'Rounded');
    ip.addParamValue('OuterSlotShape', 'Rounded');
    ip.addParamValue('InnerSlotLength', 0);
    ip.addParamValue('OuterSlotLength', 0);
    ip.addParamValue('AirgapLocation', 'Inside');
    
    ip.parse(NS, RI, RO, NW, NL, SW, SL, SA, CO, varargin{:});
    results = ip.Results;
    
    assert(results.InnerSlotLength + results.OuterSlotLength <= SL, 'MotorProto:slotTemplate',...
            'It is required that InnerSlotLength + OuterSlotLength <= slotLength');
    
    assert(NL + SL < 1, 'MotorProto:slotTemplate', 'It is required that NL + SL < 1');
        
    assert(SL > NL, 'MotorProto:slotTemplate', 'It is required that SL > NL');
    
    %% Draw notch
    dr = RO - RI;
    da = 2 * pi / NS;
    
    switch lower(results.AirgapLocation)
        case 'inside'
            notch = Geometry2D.draw('Sector', 'Radius',     [RI, RI + (RO-RI) * SL / 2],...
                                              'Rotation', - da * NW / 2, 'Angle', da * NW);
        case 'outside'
            notch = Geometry2D.draw('Sector', 'Radius',     [RO - (RO-RI) * SL / 2, RO],...
                                              'Rotation', - da * NW / 2, 'Angle', da * NW);
    end

	%% Draw polygonal portion of slot
    switch lower(results.AirgapLocation)
        case 'inside'
            x = RI + dr * NL;
        case 'outside'
            x = RO - dr * NL;
        otherwise
            unknownOptionError('AirgapLocation',results.AirgapLocation);
    end
    
    slotPoints = [x, 0];
    switch lower(results.InnerSlotShape)
        case 'straight'
         	dx = dr * results.InnerSlotLength;
        case 'rounded'
            dx = 0;
        otherwise
            unknownOptionError('InnerSlotShape', results.InnerSlotShape);
    end
    
    switch lower(results.AirgapLocation)
        case 'inside'
            slotPoints(2, :) = [x + dx,  tan(da / 2 * SW) * (x + dx)];
        case 'outside'
            slotPoints(2, :) = [x - dx, tan(da / 2 * SW) * (x - dx)];
    end
    
    dx = dr * SL;
    
    switch lower(results.AirgapLocation)
        case 'inside'
            slotPoints(4, :) = [x + dx, 0];
        case 'outside'
            slotPoints(4, :) = [x - dx, 0];
    end
    
    switch lower(results.OuterSlotShape)
        case 'straight'
            dx = dx - dr * results.OuterSlotLength;
        case 'rounded'
            dx = dx - 0;
        otherwise
            unknownOptionError('InnerSlotShape', results.OuterSlotShape);
    end
    
    switch lower(results.AirgapLocation)
        case 'inside'    
            m1 = tan(da / 2 * SA);
            b1 = slotPoints(2, 2) - m1 * slotPoints(2, 1);
            slotPoints(3, :) = [x + dx, m1 * (x + dx) + b1];
        case 'outside'    
            m1 = tan(da / 2 * SA);
            b1 = slotPoints(2, 2) - m1 * slotPoints(2, 1);
            slotPoints(3, :) = [x - dx, m1 * (x - dx) + b1];
    end

    %% Adjust slot geometry for inner slot curvature
    if strcmpi(results.InnerSlotShape, 'rounded')
        switch lower(results.AirgapLocation)
            case 'inside'
                m2 = tan(pi / 4 + (da / 2 * SA) / 2);
                b2 = slotPoints(1, 2) - m2 * slotPoints(1, 1);
            case 'outside'
                m2 = -tan(pi / 4 + (da / 2 * SA) / 2);
                b2 = slotPoints(1, 2) - m2 * slotPoints(1, 1);
        end
        
        x3 = (b1 - b2) / (m2 - m1);
        y3 = m1 * x3 + b1;
        
        yi = 0;
        if m1 ~= 0
            m3 = - 1 / m1;
            b3 = y3 - m3 * x3;
            xi = - b3 / m3;
        else
            xi = x3;
        end
        
        switch lower(results.AirgapLocation)
            case 'inside'
                ri  = xi - slotPoints(1, 1);
                ai  = abs(atan2(yi - y3, xi - x3));
                rot = pi - ai;
            case 'outside'
                ri  = hypot(x3 - xi, y3 - yi);
                ai  = pi-abs(atan2(yi - y3, xi - x3));
                rot = -ai;
        end
        slotPoints(2, :) = [x3, y3];
        
        innerSlot = Geometry2D.draw('Sector', 'Radius', [0, ri], 'Position', [xi, yi], 'Angle', 2 * ai, 'Rotation', rot);
    end
    
    %% Adjust slot geometry for outer slot curvature
    if strcmpi(results.OuterSlotShape, 'rounded')
        switch lower(results.AirgapLocation)
            case 'inside'
                m2 = - tan(pi / 4 + (da / 2 * SA) / 2);
                b2 = slotPoints(4, 2) - m2 * slotPoints(4, 1);
            case 'outside'
                m2 =   tan(pi / 4 + (da / 2 * SA) / 2);
                b2 = slotPoints(4, 2) - m2 * slotPoints(4, 1);
        end
        
        x3 = (b1 - b2) / (m2 - m1);
        y3 = m1 * x3 + b1;
        
        yo = 0;
        if m1 ~= 0
            m3 = - 1 / m1;
            b3 = y3 - m3 * x3;
            xo = - b3 / m3;
        else
            xo = x3;
        end
        
       	switch lower(results.AirgapLocation)
            case 'inside'
                ro  = hypot(x3 - xo, y3 - yo);
                ao  = pi - abs(atan2(yo - y3, xo - x3));
                rot = -ao;
            case 'outside'
                ro  = abs(xo - slotPoints(4,1));
                ao  = abs(atan2(yo - y3, xo - x3));
                rot = pi -ao;
        end
        slotPoints(3, :) = [x3, y3];
        
        outerSlot = Geometry2D.draw('Sector', 'Radius', [0, ro], 'Position', [xo, yo], 'Angle', 2 * ao, 'Rotation', rot);
    end
    
    %% Make all slot points counter clockwise
    slotPoints(5:6, :) =   slotPoints([3 2], :) * [1 0;0 -1];
    
    if strcmpi(results.AirgapLocation,'inside')
        slotPoints(:, 2)   = - slotPoints(:, 2);
    end
    
    %% Construct complete slot object
    slot = Geometry2D.draw('Polygon2D', 'Points', slotPoints,'PlotStyle',{'w'});
    
    if strcmpi(results.InnerSlotShape, 'rounded') && strcmpi(results.OuterSlotShape, 'rounded')
        slot = slot + [notch, innerSlot, outerSlot];
    elseif strcmpi(results.InnerSlotShape, 'rounded')
        slot = slot + [notch, innerSlot];
    elseif strcmpi(results.OuterSlotShape, 'rounded')
        slot = slot + [notch, outerSlot];
    else
        slot = slot + notch;
    end
    
    %% Separate into conductor/nonconductor regions
    if isempty(CO) || all((CO == 0))
        CG  = slot;
        NCG = slot.empty(1,0);
    else
        switch lower(results.AirgapLocation)
            case 'inside'
                if strcmpi(CO, 'auto')
                    CO = (slotPoints(2, 1) - RI) / dr;
                end
                conductorWindowPoints = [RI + dr * CO, max(slotPoints(:, 2)) * 2;
                                         RI + dr * CO, min(slotPoints(:, 2)) * 2;
                                         RO          , min(slotPoints(:, 2)) * 2;
                                         RO          , max(slotPoints(:, 2)) * 2];
            case 'outside'
                if strcmpi(CO, 'auto')
                    CO = (RO - slotPoints(2, 1)) / dr;
                end
                conductorWindowPoints = [RO - dr * CO, min(slotPoints(:, 2)) * 2;
                                         RO - dr * CO, max(slotPoints(:, 2)) * 2;
                                         RI          , max(slotPoints(:, 2)) * 2;
                                         RI          , min(slotPoints(:, 2)) * 2];
        end
        conductorWindow = Geometry2D.draw('Polygon2D', 'Points', conductorWindowPoints, 'PlotStyle',{'w'});
        
        CG  = slot * conductorWindow;
        NCG = slot - conductorWindow;
    end
end

function unknownOptionError(optField, optValue)
    error('MotorProto:slotTemplate', 'The value %s is not a valid value for option %s', optValue, optField);
end