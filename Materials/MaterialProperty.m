classdef MaterialProperty < matlab.mixin.Heterogeneous
    %MaterialProperties.m An abstract class for defining material properties
    %
    % MaterialProperty properties:
    %   Name                - Name of the material
    %   Color               - Default color of the material
    %   Linear              - Indicates if the material has linear magnetic properties
    %   Conductivity        - Temperature independent conductivity
    %   RemanentFluxDensity - Magnitude of the material's Remanent flux density
    %   Axis                - Alignment of the RemanentFluxDensity
    %   Polarity            - RemanentFluxDensity direction along the Axis
    %   MagnetizationType   -
    %   MagnetizationAngle  -
    %   MagnetizationAxis   -
    % 
    % MaterialProperty methods:
    %   plot            - Plots various material property curves
    %   calculateHData  -
    %   calculateBData  -
    %   calculateMData  -
  	%   magntiudeH      -
    %   magnitudeB      -
    %   magnitudeM      -
    %   vectorM         -
    %   magnitudeMr     -
    %   vectorMr        -
	%   rotate          - Rotates the Axis of the RemnanentFluxDensity
    %   reversePolarity - Changes the direction of the RemnanentFluxDensity along the Axis
    %   sigma           -
    %   elementSigma    -
    %
    % See also MotorProto
    
%{
properties
    %Name - 
    %
    % See also MaterialProperty
    Name;
    
    %Color - 
    %
    % See also MaterialProperty
    Color;
    
    %Linear - 
    %
    % See also MaterialProperty
    Linear;
    
    %Conductivity - 
    %
    % See also MaterialProperty
    Conductivity;
    
    %RemanentFluxDensity - 
    %
    % See also MaterialProperty
    RemanentFluxDensity;
    
    %Axis - 
    %
    % See also MaterialProperty
    Axis;
    
    %Polarity - 
    %
    % See also MaterialProperty
    Polarity;
    
    %MagnetizationType - 
    %
    % See also MaterialProperty
    MagnetizationType;
    
    %MagnetizationAngle - 
    %
    % See also MaterialProperty
    MagnetizationAngle;
    
    %MagnetizationAxis - 
    %
    % See also MaterialProperty
    MagnetizationAxis;
%}
    
    properties (Abstract, SetAccess = protected)
        Description
        Color

        Linear
        Conductivity
        Density
        RemanentFluxDensity
    end
    
    properties (SetAccess = private)
        Polarity = true;
    end
    
    properties (SetAccess = protected)
     	%% Permanent Magnets
        MagnetizationType  = 'Parallel';
        MagnetizationAngle = 0;
        MagnetizationAxis  = [0 0];
        
        %% Local Material Coordinate Axis
        Axis
    end
    
    methods
        %% Constructor
        function this = MaterialProperty(varargin)
            if nargin~=0
                nVarargin = numel(varargin);
                for i = 1:2:nVarargin
                    this.(varargin{i}) = varargin{i+1};
                end
            end
            this.Axis = LocalAxis;
        end
        
        %% Rebuild
        function this = rebuild(this)
            %rebuild - 
            %
            % See also MaterialProperty

            this.Axis                      = rebuild(this.Axis);
            this.RemnantMagnetizationAngle = rebuild(this.RemnantMagnetizationAngle);
            this.RemnantMagnetizationAxis  = rebuild(this.RemnantMagnetizationAxis);
        end
        
        %% Plot Method
        function gHandleOut = plot(this, typeIn)
            %plot - Plots various material property curves
            % plot(M, T) use the data of MaterialProperty object M to construct
            % a plot of the type specified in T.
            %
            % Plot Types:
            %	'bh'       - B-H curve.
            %   'dbdh'     - Derivative of the B-H curve.
            %   'mb'       - Magnetization (M-B) curve.
            %   'dmdb'     - Derivative of the M-B curve.
            %   'coreloss' - Core loss interpolation
            %
            %   Example:
            %   M = Arnon5;
            %   figure;
            %   plot(M, 'bh');
            %   plot(M, 'coreloss');
            %
            % See also MaterialProperty

            if nargin < 2
                typeIn = 'bh';
            end
            
            switch lower(typeIn)
                case {'bh', 'hb'}
                    gHandleOut = bhCurve(this);
                case {'db', 'dbdh'}
                    gHandleOut = dBdHCurve(this);
                case {'mb', 'bm'}
                    gHandleOut = mbCurve(this);
                case {'dm', 'dmdb'}
                    gHandleOut = dMdBCurve(this);
                case 'coreloss'
                    plotCoreLossCurves(this);
                otherwise
                    warning('MotorProto:MaterialProperty:plot', 'Plot type %s not known', typeIn);
            end
        end
    end
    
    methods (Static, Access = protected, Sealed)
        function defaultMaterial = getDefaultScalarObject
            defaultMaterial = Air;
        end
    end
    
    methods (Static)
        %% Static Calculations
        function mData = calculateMData(hData, bData)
            %calculateMData - 
            %
            % See also MaterialProperty

           	mData = bData / mu_o - hData;
        end
        
        function hData = calculateHData(bData, mData)
            %calculateHData - 
            %
            % See also MaterialProperty

            hData = bData / mu_o - mData;
        end
        
        function bData = calculateBData(hData, mData)
            %calculateBData - 
            %
            % See also MaterialProperty

            bData = mu_o * (hData + mData);
        end
    end
    
    methods (Sealed)
        %% Remanent Flux Density Direction Functions
        function varargout = rotate(this, angleIn, axisIn)
            %rotate - Rotates the Axis of the RemnanentFluxDensity
            % M = rotate(M, A, P) rotates the MaterialProperty objects in the
            % array M by the angle A (in radians) about the position P. The
            % center of the rotation is given by the coordinate (P(1), P(2)).
            %
            %   Example: Rotate a Permanent Magnet's material axis            
            %   figure;
            %   hold on;
            %
            %   PM          = PermanentMagnetExampleMaterial;
            %   [m, mx, my] = PM.vectorMr;
            %   c           = PM.Axis.Position;
            %   quiver(c(1), c(2), mx / m, my / m);
            %
            %   PM          = rotate(PM, pi / 2, [0.5, 0.5]);
            %   [m, mx, my] = PM.vectorMr;
            %   c           = PM.Axis.Position;
            %   quiver(c(1), c(2), mx / m, my / m);
            %
            %   legend('Original', 'Rotated');
            %
            % See also MaterialProperty
            
            [this.Axis] = rotate([this.Axis], angleIn, axisIn);   
            
            if nargout == 1
                varargout = {this};
            else
                varargout = num2cell(this); 
            end
        end
        
        function varargout = reversePolarity(this)
            %reversePolarity - 
            %
            % See also MaterialProperty

            newPolarity     = num2cell(~[this.Polarity]);
            [this.Polarity] = deal(newPolarity{:});
            varargout       = num2cell(this);
        end
        
        %% Material Property Functions
        function [Mx, My, dMxdBx, dMydBy, dMydBx, dMxdBy] = vectorM(this, Bx, By)
            %vectorM - 
            %
            % See also MaterialProperty

            bSq                = Bx.^2 + By.^2;
            b                  = sqrt(bSq);
            [m, dMdB]          = this.magnitudeM(b);
           	mDivB              = m ./ b;
            
            isZero             = (b == 0);
            i                  = find(isZero,1);
            if ~isempty(i)
                mDivB(isZero)  = dMdB(i);
            end
            
            Mx                 = Bx .* mDivB;
            My                 = By .* mDivB;
            
            bFactor            = (dMdB - mDivB) ./ (bSq);
            bFactor(isZero)    = 0;
            
            dMxdBx             = Bx.^2  .* bFactor + mDivB;
            dMxdBy             = Bx.*By .* bFactor;
            dMydBx             = dMxdBy;
            dMydBy             = By.^2  .* bFactor + mDivB;
        end
        
        function Mr = magnitudeMr(this)
            %magnitudeMr - 
            %
            % See also MaterialProperty

            Mr = [this.RemanentFluxDensity] / mu_o;
        end
        
        function [M, Mx, My] = vectorMr(this)
            %vectorMr - 
            %
            % See also MaterialProperty

            M   = [this.magnitudeMr].';

            N   = numel(this);
            ang = zeros(N,1);
            for i = 1:N
                ang(i) = this(i).MagnetizationAngle + this(i).Axis.Rotation;
                if ~this(i).Polarity
                    ang(i) = ang(i) + pi;
                end
            end
            
            Mx = M .* cos(ang);
            My = M .* sin(ang);
        end
        
        function s = sigma(this)
            N = numel(this);
            s = zeros(1, N);
            for i = 1:N
                s(i) = elementSigma(this(i));
            end
        end
        
        function d = density(this)
            N = numel(this);
            d = zeros(1, N);
            for i = 1:N
                d(i) = elementDensity(this(i));
            end
        end
    end

    methods (Abstract)
        [h, dHdB] = magnitudeH(this, b);
        [m, dMdB] = magnitudeM(this, b);
        s         = elementSigma(this, T);
    end
end

function gHandleOut = bhCurve(this)
    if all(this.HData == 0) && all(this.BData == 0)
        b = [0 2].';
    else
        b = linspace(0, max(this.BData)*1.1, 1000).';
    end
    m = this.magnitudeM(b);
    h = b/mu_o - m; 
    scatter(this.HData, this.BData);hold on;
    gHandleOut = plot(h,b);
    legend('Input Data','Interpolation','Location','Best');
    xlabel('Field Intensity, H [A/m]');
    ylabel('Flux Density, B [T]');
    title('H-B Curve, Data and Interpolation');
    hold off;
end

function gHandleOut = dBdHCurve(this)
    if all(this.HData == 0) && all(this.BData == 0)
        b = [0 1].';
    else
        b = linspace(0, max(this.BData)*1.1, 1000).';
    end
    [m,dmdb] = this.magnitudeM(b);
    h        = b / mu_o - m;
    dbdh     = mu_o ./ (1 - mu_o * dmdb);
    gHandleOut = plot(h,dbdh);hold on;
    xlabel('Field Intensity, H [A/m]');
    ylabel('$\frac{\partial B}{\partial H}$ [$\frac{T-m}{A}$]', 'interpreter','latex');
    title('H-$\frac{\partial B}{\partial H}$ Curve', 'interpreter','latex');
    hold off;
end

function gHandleOut = mbCurve(this)
    if all(this.HData == 0) && all(this.BData == 0)
        b = [0 1].';
    else
        b = linspace(0, max(this.Bs)*1.5, 1000).';
    end
    m     = this.magnitudeM(b);
    
    scatter(this.BData, this.MData);hold on;
    gHandleOut = plot(b,m);
    legend('Input Data', 'Interpolation', 'Location', 'Best');
    xlabel('Flux Density, B [T]');
    ylabel('Magnetization, M [A/m]');
    title('B-M Curve, Data and Interpolation');
    hold off;
end

function gHandleOut = dMdBCurve(this)
    if all(this.HData == 0) && all(this.BData == 0)
        b = [0 1].';
    else
        b = linspace(0, max(this.Bs)*1.5, 1000).';
    end
    [~, dmdb] = this.magnitudeM(b);
    gHandleOut = plot(b, dmdb);
    hold on;
    xlabel('Flux Density, B [A/m]');
    ylabel('$\frac{\partial M}{\partial B}$ [$\frac{A}{T-m}$]', 'interpreter','latex');
    title('B-$\frac{\partial M}{\partial B}$ Curve', 'interpreter','latex');
    hold off;
end

function plotCoreLossCurves(this)
    p = this.CoreLossData(1, :);
    B = this.CoreLossData(3, :);
    figure;
    hold on;
    scatter(B, p, 'o');
    
    p = [min(p)*0.9, max(p)*1.1];
    B = [min(B)*0.9, max(B)*1.1];
    f = unique(this.CoreLossData(2,:));
    c = this.CoreLossCoefficients;
    N = numel(f);
    
    for i = 1:N
        x = linspace(B(1),B(2),1000);
        y = c(1).*(f(i).^c(2)).*(x.^c(3));
        I = (y < p(2));
        x = x(I);
        y = y(I);
        
        plot(x,y);
        
        xc = sqrt(x(1)*x(end));
        yc = sqrt(y(1)*y(end));
        text(xc,yc,[num2str(f(i)),' Hz']);
    end
    
    grid on;
    legend('Data','Interpolation')
    ylabel('Loss Density [W / m^3]');
    xlabel('Flux Density [T]');
    set(gca,'XScale','log');
    set(gca,'YScale','log');
end