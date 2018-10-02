%% Concentrated_Winding_SMPM.m

%%% Call this part once %%%
clear all;
close all;

%% Initialize the toolbox
simulation = MotorProto('Concentrated Winding SMPM');

%% Add components to the model
model = simulation.Model;

%%% Put the rest within an optimizaiton loop %%%
%% Define General Machine Parameters
f_r               = 70;
nPoles            = 50;
nTeeth            = 60;

stackLength       = 0.14; %abs

statorInnerRadius = 0.1405; %abs
statorBILength    = 0.01;   %abs
statorBICutRadius = 0.005;  %abs

toothYokeLength   = 0.02; %abs
toothYokeWidth    = 0.5;  %pct theta
toothFaceLength   = 0.003;%abs
toothFaceChamfer  = 0.5;  %pct toothFaceLength
toothGapWidth     = 0.01; %pct theta
turnsPerTooth     = 1;    %abs

slotFilletSize1   = 0.5;  %pct radius
slotFilletSize2   = 1;    %pct radius
slotPadding       = 0.001;%abs
slotPackingFactor = 0.5;  %pct conductor

airgapLength      = 0.001;%abs

pmLength          = 4e-3; %abs
pmEmbrace         = 0.94; %pct theta
pmFilletSize      = 0.5;  %pct radius

rotorBILength     = 0.013;%abs
rotorBICutLength  = 0.007;%abs
rotorBICutWidth   = 0.9;  %pct

statorIronMaterial = Steel1010;
rotorIronMaterial  = Steel1010;
pmMaterial         = NdFe35;

[model, stator, rotor] = make_CW_SMPM_Machine(model,f_r,nPoles,nTeeth,stackLength,turnsPerTooth,statorInnerRadius, statorBILength, statorBICutRadius, toothYokeLength, toothYokeWidth, toothFaceLength, toothFaceChamfer, toothGapWidth, slotFilletSize1, slotFilletSize2, slotPadding, slotPackingFactor,airgapLength, pmLength, pmEmbrace, pmFilletSize,rotorBILength, rotorBICutWidth, rotorBICutLength, statorIronMaterial, rotorIronMaterial, pmMaterial);

%% Set mesh parameters
mesh = simulation.Mesh;
mesh(1).MaximumElementSize = (pi*statorInnerRadius/nTeeth-2*slotPadding)/6;
mesh(1).MaximumAirgapEdgeLength = [inf, airgapLength/2];
mesh(2).MaximumElementSize = pmLength / 3;
mesh(2).MaximumAirgapEdgeLength = [airgapLength/2,1];

%% Voltage Source
% stator.SourceType = 'VoltageSource';
% stator.Circuits.HarmonicNumbers    = 1;
% stator.Circuits.HarmonicAmplitudes = 708;
% stator.Circuits.HarmonicPhases     = -2 * pi * 101 / 360;

%% Current Source
stator.Layers = 2;
stator.Turns  = 1;
stator.SourceType = 'CurrentSource';
stator.Circuits.HarmonicNumbers    = 1;
stator.Circuits.HarmonicAmplitudes = 2e6*stator.Slot.Shape.area / stator.Layers / stator.Turns;
stator.Circuits.HarmonicPhases     = pi/6;

%% Simulaiton Time Points
timePointsPerPeriod = 10;

%% Static Simulation
simulation.configureAlgorithm('Static', 'TimePoints', timePointsPerPeriod, 'Verbose', true);

%% Dynamic Simulation
% stator.CouplingType = CouplingTypes.Dynamic;
% stator.Slot.ConductorType = 'Circular';
% stator.Slot.Conductor.ConductorDiameter   = 2.0*sqrt(0.5*stator.Slot.Shape.area/stator.Turns/pi/stator.Layers)*0.9 / 2;
% stator.Slot.Conductor.InsulationThickness = 2.0*sqrt(0.5*stator.Slot.Shape.area/stator.Turns/pi/stator.Layers)*0.1 / 2;
% rotor.InputRegions(1).Dynamics = DynamicsTypes.Floating; %PM Dynamics
% simulation.configureAlgorithm('ShootingNewton', 'TimePoints', timePointsPerPeriod, 'RungeKuttaStages', 2, 'StorageLevel', 3, 'Verbose', true,'ShootingTolerance',1e-4);

%% Build Model and mesh
model.build;
mesh.build;

solution = simulation.run;

%% Plotting
% solution.plot('A','Time',1);
% solution.plot('B','Time',1);
% solution.plot('H','Time',1);
% solution.plot('H','Harmonic',0);
% solution.plot('A','Harmonic',[0, model.TemporalSubharmonics]);
% solution.plot('B','Harmonic',[0, model.TemporalSubharmonics]);
solution.plot('LossDensity', 'UseSinglePlot', true, 'DataFunction', @(x)(log10(x)), 'DataFunctionString', 'log_{10}');
% solution.plot('J','Harmonic',model.TemporalSubharmonics);
% solution.plot('J','Time',1);
% solution.plot('E','Time',1);
% 
% solution.plot('FluxLinkage','Time');
% solution.plot('FluxLinkage','Harmonic');
solution.plot('Torque','Time');
solution.plot('Torque','Harmonic');
solution.plot('Voltage','Time');
solution.plot('Voltage','Harmonic');
% solution.plot('Current','Time');
% solution.plot('Current','Harmonic');

%% Data
% solution.getContinuumVariableData('H','Time',1)
% H = getPMFieldIntensity(solution,pmMaterial);
% 
% torque = solution.getBulkVariableData('Torque','Time');
% torque = torque{1};
% torque = solution.getBulkVariableData('Torque','Harmonic');
% 
% flux_linkage = solution.getBulkVariableData('FluxLinkage','Time');
% flux_linkage = flux_linkage{1}{1};
% flux_linkage = solution.getBulkVariableData('FluxLinkage','Harmonic');
% 
% mass       = solution.Model.Mass;
% statorMass = solution.Model.Assemblies(1).Mass;
% rotorMass  = solution.Model.Assemblies(2).Mass;
% 
% model.plot;