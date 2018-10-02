classdef Solution
    properties (SetAccess = private)
        Date
        Parameters
        Algorithm
    end
    
    properties (Dependent, SetAccess = private)
        Model
        Matrices
        Mesh
    end
    
    methods
        %% Constructor
        function this = Solution(algorithm)
            if nargin > 0
                this.Algorithm  = algorithm;
                this.Date       = datestr(now);
            end
        end
        
        function value = get.Matrices(this)
            value = this.Algorithm.Matrices;
        end
        
        function value = get.Model(this)
            value = this.Matrices.Model;
        end
        
        function value = get.Mesh(this)
            value = this.Model.Mesh;
        end
        
        %% Plotting
     	function plot(this, funString, varargin)
            switch funString
                case 'A'
                    plotContinuumVariable(this, 'A', varargin{:});
                    
                case 'B'
                    plotContinuumVariable(this, 'B', varargin{:});
                    
                case 'E'
                    plotContinuumVariable(this, 'E', varargin{:});
                    
                case 'H'
                    plotContinuumVariable(this, 'H', varargin{:});
                    
                case {'i', 'current', 'Current'}
                    plotBulkVariable(this, 'Current', varargin{:});
                    
                case 'J'
                    plotContinuumVariable(this, 'J', varargin{:});
                    
                case {'FluxLinkage','lambda','flux linkage','Flux Linkage','fluxlinkage','fluxLinkage'};
                    plotBulkVariable(this, 'FluxLinkage', varargin{:});
                    
                case {'LossDensity','lossdensity','Loss Density','loss density'}
                	plotContinuumVariable(this, 'AverageLossDensity', 'Average', 0, varargin{:});
                    
                case {'ConductionLossDensity','conductionlossdensity','Conduction Loss Density','conduction loss density'}
                    plotContinuumVariable(this, 'AverageConductionLossDensity', 'Average', 0, varargin{:});
                    
                case {'CoreLossDensity','corelossdensity','Core Loss Density','core loss density'}
                    plotContinuumVariable(this, 'AverageCoreLossDensity', 'Average', 0, varargin{:});
                    
                case 'M'
                    plotContinuumVariable(this, 'M', varargin{:});
                    
                case {'Voltage', 'v', 'voltage'}
                    plotBulkVariable(this, 'Voltage', varargin{:});
                    
                case {'Torque', 'tau', 'torque'}
                    plotBulkVariable(this, 'Torque', varargin{:});
                    
                case {'Mesh', 'mesh'}
                    plot(this.Mesh);
                
                otherwise
                    warning('MotorProto:Solver', 'Unknown plot type %s',funString);
            end
        end
        
        function plotContinuumVariable(this, varString, fillType, fillPoints, varargin)
            %% Get Data
            [data, dataLabels, dataText] = this.getContinuumVariableData(varString, fillType, fillPoints);
            
            [nMeshes, nFigures] = size(data);

            %% Process Inputs
            inpPar = inputParser;
            inpPar.KeepUnmatched = true;
            
            inpPar.addParamValue('Figure', -1, @(x)(all(ishandle(x))));
            inpPar.addParamValue('PlotFunction', 'fill', @(x)(any(strcmpi(x, {'fill', 'pdeplot'}))));
            inpPar.addParamValue('DataFunction', @(x)(x), @(x)(isa(x, 'function_handle')));
            inpPar.addParamValue('DataFunctionString', '', @ischar);
             
            switch lower(fillType)
                case 'time'
                    t = dataLabels;
                    inpPar.addParamValue('UseSinglePlot', true, @islogical);
                case 'harmonic'
                    data = cellfun(@(x)(abs(x)), data, 'UniformOutput', false);
                    inpPar.addParamValue('UseSinglePlot', false, @islogical);
                case 'average'
                    inpPar.addParamValue('UseSinglePlot', false, @islogical);
                otherwise
                    warning('MotorProto:Solution', 'Unknown fill type %s', fillType');
            end
            
            inpPar.parse(varargin{:});
            inpStruct = inpPar.Results;
            plotOpts  = [fields(inpPar.Unmatched).'; struct2cell(inpPar.Unmatched).'];
            
            if inpStruct.UseSinglePlot && strcmpi(fillType, 'average')
                dataLabels = sum(dataLabels);
            end
            
         	%% Get Variable Descriptions
            mp       = MotorProto;
            varName  = mp.whatIs(varString);
            titleStr = [varName, ' ',dataText]; 
            units    = mp.unitsOf(varString);
            if ~isempty(inpStruct.DataFunctionString)
                units = [inpStruct.DataFunctionString, '(', units, ')'];
            end
            
            %% Parse Plot Data
            mesh  = this.Mesh;
            
            w = zeros(1,nMeshes);
            r = cell(1,nMeshes);
            a = cell(1,nMeshes);                
            x  = cell(1,nMeshes);
            y  = cell(1,nMeshes);
            el = cell(1,nMeshes);

            nNodes = 0;
            nVals  = 0;                    
            xLimits = [inf -inf];
            yLimits = [inf -inf];
            for i = 1:nMeshes
                w(i)   = mesh(i).Assembly.AngularVelocity;
                if strcmpi(fillType, 'time');
                    r{i}   = hypot(mesh(i).X, mesh(i).Y);
                    a{i}   = atan2(mesh(i).Y, mesh(i).X);
                else
                    x{i} = mesh(i).X;
                    y{i} = mesh(i).Y;
                    
                    xLimits(1) = min(xLimits(1),min(x{i}));
                    yLimits(1) = min(yLimits(1),min(y{i}));
                    
                    xLimits(2) = max(xLimits(2),max(x{i}));
                    yLimits(2) = max(yLimits(2),max(y{i}));
                end
                
                if inpStruct.UseSinglePlot
                    el{i} = mesh(i).Elements + nNodes;
                else
                    el{i} = mesh(i).Elements;
                end
                
                nNodes = nNodes + max(numel(r{i}),numel(x{i}));
                
                nVals  = nVals  + length(data{i,1});
            end
            
            if inpStruct.UseSinglePlot
                el = [el{:}];
            end
            
            %% Parse Plotting Options
            plotPar = inputParser;
            plotPar.KeepUnmatched = true;
            switch inpStruct.PlotFunction
                case 'pdeplot'
                    plotPar.addParamValue('colormap','jet',@(x)(true));
                    if (nVals ~= nNodes) && (min(size(data{1,1})) == 1)
                        plotPar.addParamValue('xystyle','flat',@ischar);
                    end
                case 'fill'
                    plotPar.addParamValue('EdgeColor','none',@ischar);
            end
            
            plotPar.parse(plotOpts{:});
          	plotOpts  = [fields(plotPar.Results).', fields(plotPar.Unmatched).'; struct2cell(plotPar.Results).', struct2cell(plotPar.Unmatched).'];
            plotOpts  = reshape(plotOpts, 1, []);
            
            %% Do Plotting
            for i = 1:nFigures
                if strcmpi(fillType, 'time');
                    xLimits = [inf -inf];
                    yLimits = [inf -inf];
                    theta = w * t;

                    for j = 1:nMeshes
                        x{j} = r{j} .* cos(a{j} + theta(j));
                        y{j} = r{j} .* sin(a{j} + theta(j));
                        xLimits(1) = min(xLimits(1),min(x{j}));
                        yLimits(1) = min(yLimits(1),min(y{j}));

                        xLimits(2) = max(xLimits(2),max(x{j}));
                        yLimits(2) = max(yLimits(2),max(y{j}));
                    end
                end
                
                if inpStruct.UseSinglePlot    
                    xx = cell2mat(x);
                    yy = cell2mat(y);
                    dd = cell2mat(data(:,i));
                    
                    if inpStruct.Figure == -1
                        f = figure;
                    else
                        f = figure(inpStruct.Figure(i));
                    end
                    hold on;
                   
                    dd = inpStruct.DataFunction(dd);
                    switch inpStruct.PlotFunction
                        case 'pdeplot'
                            pdeplot([xx;yy], [], el, 'xydata', dd, plotOpts{:});
                        case 'fill'           
                            if nVals == nNodes
                                fill(xx(el), yy(el), dd(el), plotOpts{:});
                            else
                            	fill(xx(el), yy(el), dd.', plotOpts{:});
                            end
                    end
                    
                    if inpStruct.Figure == -1
                        axis equal;
                        xlim([min(min(xx)) max(max(xx))]);
                        ylim([min(min(yy)) max(max(yy))]);
                    else
                        xlim([min([min(xx),xlim]) max([max(xx),xlim])]);
                        ylim([min([min(yy),ylim]) max([max(yy),ylim])]);
                    end
                    
                    if numel(dataLabels) == 1
                        fName = [mp.Name, ' ', sprintf(titleStr, dataLabels)];
                    else
                        fName = [mp.Name, ' ', sprintf(titleStr, dataLabels(i))];
                    end
                    title(fName);
                    set(f,'Name', fName);
                    
                    xlabel('X [m]');
                	ylabel('Y [m]');
                    grid on;
                    
                    cb = colorbar;
                    ylabel(cb, units);
                else
                    for j = 1:nMeshes
                        if inpStruct.Figure == -1
                            f = figure;
                        else
                            f = figure(inpStruct.Figure((i-1)*nMeshes + j));
                        end
                        hold on;
                        
                        dd = inpStruct.DataFunction(data{j,i});
                        switch inpStruct.PlotFunction
                            case 'pdeplot'
                                pdeplot([x{j};y{j}], [], el{j}, 'xydata', dd, plotOpts{:});
                            case 'fill'
                                if nVals == nNodes
                                    fill(x{j}(el{j}), y{j}(el{j}), dd(el{j}), plotOpts{:});
                                else
                                    fill(x{j}(el{j}), y{j}(el{j}), dd.', plotOpts{:});
                                end
                        end
                        
                        if inpStruct.Figure == -1
                            xlim([min(min(x{j})) max(max(x{j}))]);
                            ylim([min(min(y{j})) max(max(y{j}))]);
                        else
                            xlim([min([min(x{j}),xlim]) max([max(x{j}),xlim])]);
                            ylim([min([min(y{j}),ylim]) max([max(y{j}),ylim])]);
                        end
                        
                        axis equal;
                        if numel(dataLabels) == 1
                            fName = [mp.Assemblies(j).Name, ' ', sprintf(titleStr, dataLabels)];
                        elseif numel(dataLabels) == nFigures
                            fName = [mp.Assemblies(j).Name, ' ', sprintf(titleStr, dataLabels(i))];
                        else
                            fName = [mp.Assemblies(j).Name, ' ', sprintf(titleStr, dataLabels((i-1) * nMeshes + j))];
                        end
                        
                        title(fName);
                        set(f,'Name', fName);
                        
                        xlabel('X [m]');
                        ylabel('Y [m]');
                        
                        grid on;
                        
                        cb = colorbar;
                        ylabel(cb, units);
                    end
                end
            end
        end
        
    	function plotBulkVariable(this, varString, plotType, varargin)
            lineStyle = {'-',':','-.','--'};
            lineColor = {'b', 'g', 'r', 'k', 'm', 'c', 'y', };
            
            inpPar = inputParser;
            inpPar.KeepUnmatched = true;
          	inpPar.addParamValue('DataFunction', @(x)(x), @(x)(isa(x, 'function_handle')));
            inpPar.parse(varargin{:});
            inpStruct = inpPar.Results;
            
            %% Get Data
            [data, dataLabels, dataTitles] = this.getBulkVariableData(varString, plotType);                          
            nFigures = numel(data);
            t        = this.Algorithm.Times;
            Nt       = numel(t);
            varName  = MotorProto.whatIs(varString);
            varUnits = MotorProto.unitsOf(varString);
            
            for i = 1:nFigures
                figure;
                hold on;
                
                if iscell(data{i})
                    nPlots = numel(data{i});
                else
                    nPlots = 1;
                end
                
                switch lower(plotType)
                    case 'time'
                        for j = 1:nPlots
                            if iscell(data{i})
                                y = data{i}{j};
                            else
                                y = data{i};
                            end
                            
                            y = inpStruct.DataFunction(y);
                            plot(t, y, [lineStyle{mod(j-1,4)+1} lineColor{mod(j-1,7)+1}]);
                        end
                        xlim([min(t) max(t)]);
                    	xlabel('Time [s]');
                        
                        ylabel([varName,' [',varUnits,']']);
                        
                        if ~isempty(dataLabels{i})
                            legend(dataLabels{i});
                        end
                        
                        titleStr = [strrep(dataTitles{i},'_',' '), ' ', varName, ' Waveforms'];
                        title(titleStr);
                    case 'harmonic'
                        Nh  = Nt - 1;
                        h   = 0:floor((Nh-1)/2);
                        mag = zeros(1, numel(h));
                        pha = zeros(1, numel(h));
                        for j = 1:nPlots
                            if iscell(data{i})
                                mag(j, :) = abs(data{i}{j}(h+1));
                                pha(j, :) = angle(data{i}{j}(h+1)) * 180 / pi;
                            else
                                mag(j, :) = abs(data{i}(h+1));
                                pha(j, :) = angle(data{i}(h+1)) * 180 / pi;
                            end
                            mag(j, 2:end) = 2 * mag(j, 2:end);
                        end
                        
                        sameMagnitude = true;
                        j = 1;
                        while j < nPlots && sameMagnitude
                            sameMagnitude = sameMagnitude && max(abs(mag(j,:) - mag(j+1,:))) < sqrt(eps) * max(max(abs(mag([j,j+1],:))));
                            j = j + 1;
                        end
                        
                        %% Zero out phases for negligible harmonics
                        magIsZero      = mag < sqrt(eps) * max(max(mag));
                        pha(magIsZero) = 0;
                        
                        subplot(2,1,1);hold on;
                        if sameMagnitude
                            bar(h, mag(1,:).');
                        else
                            bar(h, mag.', 'Grouped');
                            if ~isempty(dataLabels{i})
                                legend(dataLabels{i});
                            end
                        end
                        
                        titleStr = [strrep(dataTitles{i},'_',' '), ' ', varName, ' Harmonic Magnitudes'];
                        title(titleStr);
                        xlim([min(h)-1 max(h)+1]);
                        ylabel([varName,' [',varUnits,']']);
                        
                        subplot(2,1,2);hold on;
                        bar(h, pha.');
                        if sameMagnitude && ~isempty(dataLabels{i})
                            legend(dataLabels{i});
                        end
                        xlabel('Harmonic Number [n]');
                        titleStr = [strrep(dataTitles{i},'_',' '), ' ', varName, ' Harmonic Phases'];
                        title(titleStr);
                        xlim([min(h)-1 max(h)+1]);
                        ylabel('Degrees');
                end
            end
        end
        
        %% Data Access Interfaces
        function [data, dataLabels, dataText] = getContinuumVariableData(this, varString, fillType, fillPoints)
            [data, dataLabels, dataText] = this.Matrices.(varString)(this.Algorithm, fillType, fillPoints);
        end

        function [data, dataLabels, dataTitle] = getBulkVariableData(this, varString, dataType)
            if nargin < 3
                dataType = 'Time';
            end
            [data, dataLabels, dataTitle] = this.Matrices.(varString)(this.Algorithm, dataType);
        end
        
      	function varargout = getData(this, varString, varargin)
            if nargin == 2
                varargin{1} = 'default';
            end
            
            varargout = {this.Matrices.(varString)(this.Algorithm, varargin{:})};
        end
    end
end