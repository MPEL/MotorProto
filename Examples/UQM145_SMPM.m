%% UQM145_SMPM.m
clear all;
close all;

%% Initialize the toolbox
simulation = MotorProto('UQM145_SMPM');

%% Add components to the model
model  = simulation.Model;
rotor  = model.newAssembly('SMPM Rotor','SynchronousRotor');
stator = model.newAssembly('SMPM Stator','Stator');

%% Define General Machine Parameters
nPoles            = 18;
nTeethPerPhase    = 2;
nTeeth            = 3 * nPoles * nTeethPerPhase;
nTurnsPerSlot     = 6;
nParallelPaths    = 3;
len               = 0.1428877 * 2 / 3; %modify length to give effect number of turns = 4
statorOuterRadius = 0.1250908;
statorInnerRadius = 0.0986028;
rotorOuterRadius  = 0.0971550;
rotorInnerRadius  = 0.0853906;
f_r               = 8000/60;
f_e               = f_r*nPoles / 2;
T                 = 1/f_e;

%% Define Stator Geometry and Material Properties
stator.ElectricalFrequency = f_e;
stator.Length              = len;
stator.Poles               = nPoles;
stator.Teeth               = nTeeth;
stator.InnerRadius         = statorInnerRadius;
stator.OuterRadius         = statorOuterRadius;
stator.DefaultMaterial     = Iron;
stator.SourceType          = SourceTypes.CurrentSource;
stator.CouplingType        = CouplingTypes.Dynamic;
stator.WindingType         = WindingTypes.Distributed;
stator.ParallelPaths       = nParallelPaths;

%% Slot
stator.Slot.Turns = nTurnsPerSlot;

%% Stranded Conductors
stator.Slot.ConductorType                 = ConductorTypes.Circular;
stator.Slot.Conductor.ConductorDiameter   = 1.0e-3 * 0.9 * 2 / (2^(1.0));
stator.Slot.Conductor.InsulationThickness = 0.10e-3 * 1.1 * 2 / (2^(1.0));

% stator.Slot.Conductor.ConductorDiameter = 0.7239*1e-3;
% stator.Slot.Conductor.InsulationThickness = 0.04405*1e-3;

%% Solid Conductors
% stator.Slot.ConductorType           = 'Homogenized';
% stator.Slot.Conductor.PackingFactor = 0.5;

%% Define slot geometry
Bsat = 2.0;
Bmag = 1.23;

slotWidth   = 1-statorInnerRadius/rotorOuterRadius*Bmag/Bsat;
slotLength  = 1-statorInnerRadius*(1-slotWidth)*2*pi/nPoles*(3*nTeethPerPhase-1)/(3*nTeethPerPhase)/(statorOuterRadius-statorInnerRadius)/2;
notchWidth  = 0.1795;
notchLength = 0.01263;
[slotOutline, slotNotch] = slotTemplate(nTeeth, statorInnerRadius, statorOuterRadius, notchWidth, notchLength, slotWidth, slotLength, 1, 'auto', 'InnerSlotShape','rounded','OuterSlotShape','rounded');

stator.Slot.Shape        = slotOutline;
stator.ConductorMaterial = Copper;

stator.addRegion('slot', slotNotch, Air, DynamicsTypes.Static); 

%% Set Rotor Parameters
rotor.Poles               = nPoles;
rotor.Length              = len;
rotor.ElectricalFrequency = f_r * nPoles / 2;
rotor.InnerRadius         = rotorInnerRadius;
rotor.OuterRadius         = rotorOuterRadius;
rotor.DefaultMaterial     = Iron;
rotor.OperatingMode       = OperatingModes.Synchronous;
rotor.InitialAngle        = pi/nPoles*0;
rotor.BackironType        = BackironTypes.Laminated;

%% Create Rotor Permanent Magnet
pmRing     = 0.775e-3;
pmWidth    = 4e-3+pmRing;
pmEmbrace  = (1-0.11);
pmLength   = 2 * (rotorOuterRadius - pmWidth) * tan(2 * pi / nPoles / 2) * pmEmbrace;
pmPosition = [statorOuterRadius / 2 + rotorOuterRadius - pmWidth / 2, 0];

pmBody = Geometry2D.draw('Rect', 'Width', pmLength, 'Length', pmWidth + statorOuterRadius, 'Base', 'Center', 'Position', pmPosition, 'PlotStyle', {'m'});
pmTrim = Geometry2D.draw('Sector', 'Radius', [rotorInnerRadius, rotorOuterRadius-pmRing], 'Angle', 2 * pi / nPoles, 'Rotation', - pi / nPoles);
permanentMagnet = pmBody * pmTrim;
rotor.addRegion('pm', permanentMagnet,  NdFe35, DynamicsTypes.Floating);

%% Trim Iron Between Magnets
trim1 = [rotorOuterRadius - pmWidth, pmLength / 2];
trim2 = [sqrt(statorOuterRadius^2 - pmLength^2 / 4), pmLength /2];
trim3 = statorOuterRadius * [cos(pi / nPoles), sin(pi / nPoles)];

poleM  = tan( pi / nPoles);
trim4M = - trim1(1) / trim1(2);
trim4b = trim1(2) - trim4M * trim1(1);

trim4 = trim4b / (poleM - trim4M) * [1, poleM];

trimPoints = [trim1;trim2;trim3;trim4];

trimUHP = Geometry2D.draw('Polygon2D', 'Points', trimPoints, 'PlotStyle', {'w'});
trimUHP = trimUHP * pmTrim;

trimPoints(:,2) = -trimPoints(:,2);
trimPoints      = flipud(trimPoints);

trimLHP = Geometry2D.draw('Polygon2D', 'Points', trimPoints, 'PlotStyle', {'w'});
trimLHP = trimLHP * pmTrim;

rotor.addRegion('trimUHP', trimUHP, Air, DynamicsTypes.Static);
rotor.addRegion('trimLHP', trimLHP, Air, DynamicsTypes.Static);

retainingRing = Geometry2D.draw('Sector', 'Radius', [rotorOuterRadius-pmRing, rotorOuterRadius], 'Angle', 2 * pi / nPoles, 'Rotation', - pi / nPoles,'PlotStyle',{'b'});
rotor.addRegion('ring', retainingRing, Iron, DynamicsTypes.Static);

%% Set mesh parameters
mesh = simulation.Mesh;
% mesh(1).MaximumElementSize = pmWidth / 4;
% mesh(2).MaximumElementSize = (2*pi*statorInnerRadius)*(0.5/nTeeth)*0.28;
% mesh(1).MaximumAirgapEdgeLength = [inf, 2*pi*rotorOuterRadius / nTeeth / 10];
% mesh(2).MaximumAirgapEdgeLength = [2*pi*statorInnerRadius / nTeeth / 10, inf];

%% Set Excitation
%% Voltage Source
% h = 1;
% V = 340 / sqrt(3) * exp(1i*(-pi/2 + pi/6 + pi*(-1/8+1/16-1/32+1/64-1/128-1/256)));
% 
% h = 1:2:2001;
% V = 1i * 340 / 2 * 4/pi./h .* exp(1i*(pi/(10*exp(1))*h)) .* abs(1./(1+(1i*h*f_e/120000)));

% stator.SourceType = SourceTypes.VoltageSource;
% stator.ParallelPaths = nParallelPaths;
% stator.Circuits.ElectricalFrequency = f_e;
% stator.Circuits.HarmonicNumbers     = h;
% stator.Circuits.HarmonicAmplitudes  = abs(V);
% stator.Circuits.HarmonicPhases      = angle(V);

%% Current Source
% h = 1;
% Iq = 500;
% Id = -150;
% I  = Iq*exp(1i*(-120)*pi/180) + Id*exp(1i*(-30)*pi/180);
h = 1;
I = 0;

% h = 1:2:1001;
% h(mod(h,3)==0) = [];
% I = 500*1i*(cos(pi*h/6)-cos(5*pi*h/6)) ./ (pi*h) .* exp(1i*(pi/(10*exp(1))*h)) .* abs(1./(1+(1i*h*f_e/12000)));

stator.SourceType = SourceTypes.CurrentSource;
stator.ParallelPaths = nParallelPaths;
stator.Circuits.ElectricalFrequency = f_e;
stator.Circuits.HarmonicNumbers     = h;
stator.Circuits.HarmonicAmplitudes  = abs(I);
stator.Circuits.HarmonicPhases      = angle(I);

%% Simulate
nTimePoints = 54;
%simulation.configureAlgorithm('Static',          'TimePoints', nTimePoints, 'Verbose', true);
%simulation.configureAlgorithm('ShootingNewton',  'TimePoints', nTimePoints, 'RungeKuttaStages', 3, 'StoreDecompositions', true, 'Verbose', true, 'SymmetricJacobian', true,'Adaptive',true,'AdaptiveTolerance',1e-3);
%simulation.configureAlgorithm('TPFEM',           'TimePoints', nTimePoints, 'RungeKuttaStages', 3, 'StoreDecompositions', true, 'Verbose', true, 'SymmetricJacobian', true, 'Adaptive', true, 'AdaptiveTolerance', 1e-4);
simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', nTimePoints,                        'StoreDecompositions', true, 'Verbose', true,                            'Adaptive', false, 'AdaptiveTolerance', 1e-3, 'MaxGMRESIterations', 1, 'NewtonTolerance', eps);

model.build;
mesh.build;
solution = simulation.run;

%% Plotting
% solution.plot('A','Time',1);
% solution.plot('B','Time',1);
% solution.plot('H','Time',1);
% solution.plot('M','Time',1);
% solution.plot('A','Harmonic',[0, 1]);
%solution.plot('B','Harmonic',[6]);
% solution.plot('H','Harmonic',[0, 1]);
% solution.plot('M','Harmonic',[0, 1]);
% solution.plot('LossDensity', 'UseSinglePlot', true);
% solution.plot('LossDensity', 'UseSinglePlot', true, 'DataFunction', @(x)(log10(x)), 'DataFunctionString', 'log_{10}');
% solution.plot('J','Time',1);
% solution.plot('J','Harmonic',1);
% solution.plot('E','Time',1);
% solution.plot('E','Harmonic',1);

solution.plot('Flux Linkage','Time');
% solution.plot('Flux Linkage','Harmonic');
solution.plot('Torque','Time');
% solution.plot('Torque','Harmonic');
solution.plot('Voltage','Time');
% solution.plot('Voltage','Harmonic');
solution.plot('Current','Time');
% solution.plot('Current','Harmonic');
% 
% t = solution.Algorithm.Times;
% figure;plot(reshape([t(1:end-1);t(2:end)],1,[]),reshape([diff(t);diff(t)],1,[]));
% hold on;scatter(t(2:end),diff(t));
% figure;hist(diff(t))
% i = solution.getBulkVariableData('Current','Time');
% i = i{1};
% figure;hold on;
% plot(t,i{1});
% scatter(t,i{1},'ob');
% plot(t,i{2},'g--');
% scatter(t,i{2},'og');
% plot(t,i{3},'r-.');
% scatter(t,i{3},'or');
% % 
% % legend('A','B','C');
% % xlabel('Time [s]');
% % ylabel('Voltage [V]');
% % title('Phase Currents');
% % 
% v = solution.getBulkVariableData('Voltage','Time');
% % 
% % figure;hold on;
% % plot(t,v{1}{1})
% % plot(t,v{1}{2},'g--');
% % plot(t,v{1}{3},'r-.');
% % scatter(t,v{1}{1},'ob')
% % scatter(t,v{1}{2},'og');
% % scatter(t,v{1}{3},'or');
% % legend('A','B','C');
% % xlabel('Time [s]');
% % ylabel('Voltage [V]');
% % title('Phase Voltage');
% % 
% figure;hold on;
% plot(t,v{1}{1}-v{1}{2})
% plot(t,v{1}{2}-v{1}{3},'g--');
% plot(t,v{1}{3}-v{1}{1},'r-.');
% % scatter(t,v{1}{1}-v{1}{2},'ob')
% % scatter(t,v{1}{2}-v{1}{3},'og');
% % scatter(t,v{1}{3}-v{1}{1},'or');
% legend('AB','BC','CA');
% xlabel('Time [s]');
% ylabel('Voltage [V]');
% title('Line to Line Voltage');