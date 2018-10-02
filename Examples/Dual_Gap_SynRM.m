%% Dual_Gap_SynRM.m
%   Toy example of a multiple airgap machine

clear all;
close all;

tic

%% Initialize the toolbox
simulation = MotorProto('DualGapSRM');

%% Add components to the model
model   = simulation.Model;
iStator = model.newAssembly('SRM Inner Stator', 'Stator');
rotor   = model.newAssembly('SRM Rotor', 'SynchronousRotor');
oStator = model.newAssembly('SRM Outer Stator', 'Stator');

%% Model Parameters
Poles  = 8;
Teeth  = Poles * 3 * [2, 3];
Airgap = 1e-3;
Radii  = [0.03, 0.06, 0.06+Airgap, 0.09+Airgap, 0.09+2*Airgap, 0.12+2*Airgap];
Length = sqrt(pi)*Radii(6);
w_r    = 60;

%% Inner Stator
iStator.Poles               = Poles;
iStator.ElectricalFrequency = w_r * Poles / 2;
iStator.Teeth               = Teeth(1);
iStator.InnerRadius         = Radii(1);
iStator.OuterRadius         = Radii(2);
iStator.Length              = Length;
iStator.InitialAngle        = 0;
iStator.CouplingType        = CouplingTypes.Static;

[slotOutline, slotNotch] = slotTemplate(Teeth(1), Radii(1), Radii(2), 0.1, 0.02, 0.5, 0.5, 1, 'auto', 'InnerSlotShape', 'rounded', 'OuterSlotShape', 'rounded', 'AirgapLocation', 'outside');
iStator.Slot.Shape       = slotOutline;
iStator.addRegion('islot1', slotNotch, Air, DynamicsTypes.Static); 

%% Rotor
rotor.Poles               = Poles;
rotor.ElectricalFrequency = w_r * Poles / 2;
rotor.InnerRadius         = Radii(3);
rotor.OuterRadius         = Radii(4);
rotor.Length              = Length;

trim = Geometry2D.draw('Sector', 'Radius', [Radii(3), Radii(4)], 'Angle', 2*pi/Poles, 'Rotation', -pi/Poles);

r = [0, Radii(4) * pi / Poles * 1 / 2];
c = [Radii(4), 0];
cut1 = Geometry2D.draw('Sector', 'Radius', r, 'Angle', 2*pi, 'Rotation', 0, 'Position', c);
cut1 = cut1 * trim;
rotor.addRegion('cut1', cut1, Air, DynamicsTypes.Static);

r = [0, Radii(3) * pi / Poles * 1 / 2];
c = [Radii(3)*cos(-pi/Poles), Radii(3)*sin(-pi/Poles)];
cut2 = Geometry2D.draw('Sector', 'Radius', r, 'Angle', 2*pi, 'Rotation', 0, 'Position', c);
cut2 = cut2 * trim;
rotor.addRegion('cut2', cut2, Air, DynamicsTypes.Static);

r = [0, Radii(3) * pi / Poles * 1 / 2];
c = [Radii(3)*cos(pi/Poles), Radii(3)*sin(pi/Poles)];
cut3 = Geometry2D.draw('Sector', 'Radius', r, 'Angle', 2*pi, 'Rotation', 0, 'Position', c);
cut3 = cut3 * trim;
rotor.addRegion('cut3', cut3, Air, DynamicsTypes.Static);

%% Outer Stator
oStator.Poles               = Poles;
oStator.ElectricalFrequency = w_r * Poles / 2;
oStator.Teeth               = Teeth(2);
oStator.InnerRadius         = Radii(5);
oStator.OuterRadius         = Radii(6);
oStator.Length              = Length;
oStator.CouplingType        = CouplingTypes.Static;

[slotOutline, slotNotch] = slotTemplate(Teeth(2), Radii(5), Radii(6), 0.1, 0.02, 0.5, 0.5, 1, 'auto', 'InnerSlotShape', 'rounded', 'OuterSlotShape', 'rounded', 'AirgapLocation', 'inside');
oStator.Slot.Shape       = slotOutline;
oStator.addRegion('oslot1', slotNotch, Air, DynamicsTypes.Static);

%% Meshing
mesh = model.Mesh;
mesh(1).MaximumElementSize = (Radii(2)-Radii(1)) / 40;
mesh(2).MaximumElementSize = (Radii(4)-Radii(3)) / 40;
mesh(3).MaximumElementSize = (Radii(6)-Radii(5)) / 40;

%% Set Excitation
iStator.SourceType                  = SourceTypes.CurrentSource;
iStator.Sources.ElectricalFrequency = w_r * Poles / 2;
iStator.Sources.HarmonicNumbers     = 1;
iStator.Sources.HarmonicAmplitudes  = 100;
iStator.Sources.HarmonicPhases      = pi;

oStator.SourceType                  = SourceTypes.CurrentSource;
oStator.Sources.ElectricalFrequency = w_r * Poles / 2;
oStator.Sources.HarmonicNumbers     = 1;
oStator.Sources.HarmonicAmplitudes  = 71;
oStator.Sources.HarmonicPhases      = pi / 2;

%% Run Simulation
nTimePoints = 72;
simulation.configureAlgorithm('Static', 'TimePoints', nTimePoints, 'Verbose', true);
model.build;
mesh.build;
solution = simulation.run;

% solution.plot('A','Time',1);
% solution.plot('B','Time',1);
% solution.plot('B','Harmonic',[0,1]);
solution.plot('Torque','Time');
solution.plot('Torque','Harmonic');
% solution.plot('Flux Linkage','Time');