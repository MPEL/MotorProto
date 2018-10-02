%% IPM_Machine_Tutorial.m
clear all;
close all;
tic

%% Initialize the toolbox
simulation = MotorProto('IPM Machine Tutorial');

%% Add components to the model
model  = simulation.Model;
rotor  = model.newAssembly('IPM Rotor','SynchronousRotor');
stator = model.newAssembly('IPM Stator','Stator');

%% Define General Machine Parameters
nPoles            = 8;
nTeethPerPhase    = 3;
nTeeth            = 3 * nTeethPerPhase * nPoles;
len               = 0.18;
statorOuterRadius = 0.12;
statorInnerRadius = 0.08513;
rotorOuterRadius  = 0.085;
rotorInnerRadius  = 0.05;
w_r               = 70;

%% Define Stator Geometry and Material Properties
stator.ElectricalFrequency = w_r * nPoles / 2;
stator.Length              = len;
stator.Poles               = nPoles;
stator.Teeth               = 3 * nPoles * nTeethPerPhase;
stator.InnerRadius         = statorInnerRadius;
stator.OuterRadius         = statorOuterRadius;
stator.DefaultMaterial     = Iron;
stator.CouplingType        = CouplingTypes.Static;
stator.Slot.Turns          = 2;

%Stranded-Style Conductors
% stator.Slot.ConductorType                 = ConductorTypes.Circular;
% stator.Slot.Conductor.ConductorDiameter   = 1.75e-3;
% stator.Slot.Conductor.InsulationThickness = 0.1e-3;

%Bus-Bar Style Conductors
stator.Slot.ConductorType           = ConductorTypes.Homogenized;
stator.Slot.Conductor.PackingFactor = 0.5;

%% Define slot geometry
slotWidth   = 0.5;
slotLength  = 0.5;
notchWidth  = 1 / 2 / nTeethPerPhase / 3;
notchLength = 0.01;

[conductorOutline, innerSlot] = slotTemplate(nTeeth, statorInnerRadius, statorOuterRadius, notchWidth, notchLength, slotWidth, slotLength, 0, 'Auto');
stator.Slot.Shape             = conductorOutline;
stator.ConductorMaterial      = Copper;
                
stator.addRegion('innerSlot', innerSlot, Air, DynamicsTypes.Static); 

%% Set Rotor Parameters
rotor.Poles               = nPoles;
rotor.Length              = len;
rotor.ElectricalFrequency = w_r * nPoles / 2;
rotor.InnerRadius         = rotorInnerRadius;
rotor.OuterRadius         = rotorOuterRadius;
rotor.DefaultMaterial     = Iron;
rotor.OperatingMode       = OperatingModes.Synchronous;
rotor.InitialAngle        = 0;

%% Create Rotor Permanent Magnet
pmAngle          = 2 * pi / 3 / nPoles * 2;
pmRadPercent     = 0.9;
fbMinBridgeWidth = 0.0001;

dMax       = (rotorOuterRadius - fbMinBridgeWidth ) * cos(pmAngle / 2);
pmVolume   = 22 * (statorInnerRadius - rotorOuterRadius) * 2 * dMax * tan(pmAngle / 2);
dMin       = (rotorInnerRadius + fbMinBridgeWidth) * cos(pmAngle / 2);
d          = dMin - (dMin - dMax) * pmRadPercent;
pmLength   = 2 * d * tan(pmAngle / 2);
pmWidth    = pmVolume / pmLength;
pmPosition = [d - pmWidth / 2, 0];

permanentMagnet = Geometry2D.draw('Rect', 'Width', pmLength, 'Length', pmWidth, 'Base', 'Center', 'Position', pmPosition, 'PlotStyle', {'m'});
rotor.addRegion('pm', permanentMagnet,  NdFe35, DynamicsTypes.Floating);

%% Create Rotor Flux Barriers
rFB   = rotorOuterRadius - fbMinBridgeWidth;
pmRad = sqrt(d^2 + pmLength^2 / 4);

xFB0 = pmRad * cos(pmAngle / 2);
yFB0 = pmRad * sin(pmAngle / 2);

xFB1 = rFB * cos(pmAngle / 2);
yFB1 = rFB * sin(pmAngle / 2);

xFB2 = rFB * cos(pmAngle / 2 + pi * 1 / (3 * nTeethPerPhase * nPoles));
yFB2 = rFB * sin(pmAngle / 2 + pi * 1 / (3 * nTeethPerPhase * nPoles));

xFB4 = xFB0 - pmWidth;
yFB4 = yFB0;

rFB3 = sqrt(xFB4^2 + yFB4^2);
xFB3 = rFB3 * cos(pmAngle / 2 + pi * 1 / (3 * nTeethPerPhase * nPoles));
yFB3 = rFB3 * sin(pmAngle / 2 + pi * 1 / (3 * nTeethPerPhase * nPoles));

fbPoints = [xFB0 yFB0;
            xFB1 yFB1;
            xFB2 yFB2;
            xFB3 yFB3;
            xFB4 yFB4];
        
fb1 = Geometry2D.draw('Polygon2D', 'Points', fbPoints, 'PlotStyle', {'w'});
rotor.addRegion('fb1', fb1, Air, DynamicsTypes.Static);

fbPoints = [xFB4 -yFB4;
            xFB3 -yFB3;
            xFB2 -yFB2;
            xFB1 -yFB1;
            xFB0 -yFB0];
        
fb2 = Geometry2D.draw('Polygon2D', 'Points', fbPoints, 'PlotStyle', {'w'});
rotor.addRegion('fb2', fb2, Air, DynamicsTypes.Static);
        
%% Set mesh parameters
mesh                       = simulation.Mesh;
mesh(1).MaximumElementSize = (statorOuterRadius - rotorInnerRadius) / 40;
mesh(2).MaximumElementSize = (statorOuterRadius - rotorInnerRadius) / 40;

%% Set Excitation
stator.Sources.ElectricalFrequency = w_r * nPoles / 2;

%% Voltage Source
% stator.SourceType = SourceTypes.VoltageSource;
% stator.Sources.HarmonicNumbers    = 1:2:17;
% stator.Sources.HarmonicAmplitudes = [392.940960259865,38.7974461566293,21.1686231750374,18.5295847823860,6.54971559669156,2.95498716209424,8.02036987709044,4.85090773859384,6.58391266174923;];
% stator.Sources.HarmonicPhases     = [1.83559893815957,2.83723513902788,-2.53101267526780,-2.54878725386589,-3.09621299590694,1.63134692441761,0.313394958242182,-1.28085787831664,2.22475806066111;];

% %Current Source
stator.SourceType = SourceTypes.CurrentSource;
stator.Sources.HarmonicNumbers    = 1;
stator.Sources.HarmonicAmplitudes = 305 / sqrt(3);
stator.Sources.HarmonicPhases     = (2 * pi / 3) + 2 * pi / 3 * 1.1;

%% Configure algorithm
model.build;
mesh.build;

nSlotHarmonics = 1;
nTimePoints    = 2 * (2 * nTeethPerPhase * 3 + 1) * nSlotHarmonics + 1;
% simulation.configureAlgorithm('Static', 'TimePoints', nTimePoints, 'Verbose', true);
simulation.configureAlgorithm('ShootingNewton', 'TimePoints', nTimePoints, 'RungeKuttaStages', 2, 'StorageLevel', 3, 'Verbose', true);

solution = simulation.run;

%% Plotting
solution.plot('A','Time',1);
solution.plot('B','Time',1);
solution.plot('A','Harmonic',[0, 1]);
solution.plot('B','Harmonic',[0, 1]);
solution.plot('LossDensity', 'UseSinglePlot', true, 'DataFunction', @(x)(log10(x)), 'DataFunctionString', 'log_{10}');
solution.plot('J','Harmonic',1);
solution.plot('J','Time',1);

solution.plot('Flux Linkage','Time');
solution.plot('Flux Linkage','Harmonic');
solution.plot('Torque','Time');
solution.plot('Voltage','Time');
solution.plot('Current','Time');