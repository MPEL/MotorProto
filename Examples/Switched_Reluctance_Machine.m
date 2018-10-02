%% Switched_Reluctance_Machine.m
%   Data and model parameters from Tim Burress at ORNL

clear all;
close all;

%% Initialize the toolbox
simulation = MotorProto('SRMotor');

%% Add components to the model
model  = simulation.Model;
rotor  = model.newAssembly('SRRotor', 'SynchronousRotor');
stator = model.newAssembly('SRStator', 'Stator');

%% Model Parameters
Poles       = 4;
StatorTeeth = 12;
RotorTeeth  = 8;
f_r         = 1/8/180;

rotorInnerRadius  = 25.4*1e-3;
rotorOuterRadius  = 76.2*1e-3;
airgapLength      = 1e-3;
statorInnerRadius = rotorOuterRadius + airgapLength;
statorOuterRadius = 132*1e-3;
stackLength       = 3*25.4*1e-3;

%% Rotor
rotor.Poles               = Poles;
rotor.ElectricalFrequency = f_r;
rotor.InnerRadius         = rotorInnerRadius;
rotor.OuterRadius         = rotorOuterRadius;
rotor.Length              = stackLength;
rotor.OperatingMode       = OperatingModes.Synchronous;
rotor.InitialAngle        = 0;
rotor.DefaultMaterial     = MaxwellM19;

Krtr     = 0.65;
rtAngle  = 2*pi/(RotorTeeth*2)*Krtr*1/2; %angle of rotor tooth corner at airgap w.r.t. x-axis
rtLength = 0.75*25.4*1e-3;

t_r = [rotorOuterRadius;
       hypot(rotorOuterRadius - rtLength, rotorOuterRadius*sin(rtAngle))];

slot = Geometry2D.draw('Sector','Radius',[t_r(2), t_r(1)], 'Angle', 2*pi/Poles,'Rotation',-pi/Poles);
for i = 1:((RotorTeeth/Poles) + 1)
    tooth = Geometry2D.draw('Rect','Length',2*statorOuterRadius,'Width',2*rotorOuterRadius*sin(rtAngle),'Rotation',-pi/Poles + 2*pi/RotorTeeth * (i-1));
    slot  = slot - tooth;
end

rotor.addRegion('rotorSlot', slot, Air, DynamicsTypes.Static);

%% Outer Stator
stator.Poles               = Poles;
stator.ElectricalFrequency = f_r;
stator.Teeth               = StatorTeeth;
stator.InnerRadius         = statorInnerRadius;
stator.OuterRadius         = statorOuterRadius;
stator.Length              = stackLength;
stator.CouplingType        = CouplingTypes.Static;
stator.WindingType         = WindingTypes.Concentrated;
stator.Layers              = 2;
stator.Turns               = 30;
stator.DefaultMaterial     = MaxwellM19;

Kstr       = 1;
stAngle    = asin(Kstr*rotorOuterRadius*sin(rtAngle)/statorInnerRadius);
slotDepth  = 30*1e-3;
slotRadius = rotorOuterRadius+airgapLength+slotDepth;

slot = Geometry2D.draw('Sector','Radius',[statorInnerRadius,slotRadius], 'Angle', 2*pi/StatorTeeth,'Rotation',-pi/StatorTeeth);
tooth1 = Geometry2D.draw('Rect','Length',2*statorOuterRadius,'Width',2*statorInnerRadius*sin(stAngle),'Rotation',-pi/StatorTeeth);
tooth2 = Geometry2D.draw('Rect','Length',2*statorOuterRadius,'Width',2*statorInnerRadius*sin(stAngle),'Rotation',pi/StatorTeeth);
slot = slot-tooth1-tooth2;

coilInset = 1e-3;
gapAngle  = 2*pi/StatorTeeth-0.95*2*pi/(2*StatorTeeth)*2;
insetTrim = Geometry2D.draw('Sector','Radius',[statorInnerRadius,statorInnerRadius + coilInset],'Angle',2*pi/Poles,'Rotation',-pi/Poles);
gapTrim = Geometry2D.draw('Sector','Radius',[statorInnerRadius,statorOuterRadius],'Angle',gapAngle,'Rotation',-gapAngle/2);
slotGap = (insetTrim + gapTrim) * slot;
slot = slot - insetTrim - gapTrim;
stator.addRegion('SlotGap',slotGap, Air, DynamicsTypes.Static);

% stator.Slot.ConductorType = ConductorTypes.Circular;
% stator.Slot.Conductor.ConductorDiameter   = 2.0*sqrt(0.5*slot.area/stator.Turns/pi/stator.Layers)*0.9;
% stator.Slot.Conductor.InsulationThickness = 2.0*sqrt(0.5*slot.area/stator.Turns/pi/stator.Layers)*0.1;

stator.Slot.Shape = slot;

%% Simulate
model.build;
mesh = model.Mesh;
mesh(1).MaximumElementSize = 4e-3;
mesh(1).MaximumAirgapEdgeLength = [inf,1e-3];
mesh(2).MaximumElementSize = 4e-3;
mesh(2).MaximumAirgapEdgeLength = [1e-3,inf];

mesh.build;

N = 6;
J = 100*(1:N);
tau = cell(1,N);
nTimePoints = 19;
simulation.configureAlgorithm('Static', 'TimePoints', nTimePoints, 'TimeInterval', [1,2]/f_r/8, 'Verbose', true, 'NewtonTolerance', sqrt(eps));
% simulation.configureAlgorithm('Static', 'TimePoints', nTimePoints, 'Verbose', true, 'NewtonTolerance', sqrt(eps));
for i = 1:N
    stator.Circuits.ElectricalFrequency = 0;
    stator.Circuits.HarmonicNumbers     = [1;1;1];
    stator.Circuits.HarmonicAmplitudes  = [J(i);0;0];
    stator.Circuits.HarmonicPhases      = [0;0;0];

    solution = simulation.run;
    
    t      = solution.Algorithm.Times;
    tau{i} = solution.getBulkVariableData('Torque','Time');
    tau{i} = tau{i}{1};
    
    figure(1);hold on;
    t = t - t(1);
    h = plot(t,real(tau{i}),'-ob','LineWidth',1.5,'MarkerSize',8);
end
legend(h,'MATLAB')
xlabel('Rotor Position');
ylabel('Torque [N-m]');
grid on;

hold on;
maxwelldata = [ 0	0.009297789	0.037330167	0.054028845	0.062820637	0.065145435	0.062781879;
                10	4.301526345	16.47579304	27.31205421	33.92998625	39.85323143	45.48673093;
                20	9.216303335	34.92490462	56.44482586	69.25453923	80.62311223	91.40260989;
                30	15.57292025	56.46435728	86.38110888	104.3773955	120.2976607	135.4529438;
                40	24.752411	82.67861376	118.3278058	140.4154636	159.9722179	178.5797634;
                50	40.21391375	112.3894554	150.208384	174.7578021	196.8153042	218.0712597;
                60	63.88305458	144.2999271	181.6659212	208.1173202	232.1776613	255.4743161;
                70	78.91199799	158.7445067	194.484971	221.5834909	246.7496291	271.2264195;
                80	81.00206053	156.9460616	190.8342324	218.1253752	243.8129422	268.8227634;
                90	82.72895327	152.0753288	184.2291321	211.3419459	237.0827828	261.8609429;
                100	80.78832201	142.5002739	172.9727836	199.5986102	224.7174903	248.4285832;
                110	77.93716723	131.1336183	160.0004709	185.6078657	209.3350301	230.5765875;
                120	74.89179585	119.4865982	146.3405599	170.1718251	191.0542042	209.3921725;
                130	66.40176257	103.2144769	127.8556372	148.8497637	166.5405913	182.5233532;
                140	56.81331718	86.68597424	108.0802031	125.3855951	140.2762515	153.8036032;
                150	44.50628615	68.45566793	85.76701834	99.67752068	111.7359171	122.5741605;
                160	31.06009869	48.07608704	60.74128884	70.98432805	79.85460865	87.85833314;
                170	18.30802716	26.34770916	33.54526919	39.42561496	44.3236678	48.56921339;
                180	-0.019727154	-0.007499624	-0.009619774	0.006840295	0.017802268	0.027591094];
for i = 2:7;
    h=plot(maxwelldata(:,1),maxwelldata(:,i),'--xk','LineWidth',1.5,'MarkerSize',8);
end
ah = axes('position',get(gca,'position'),'visible','off');
legend(ah,h,'Maxwell');
% 
% solution.plot('J','Time',1);
% solution.plot('A','Time',1);
% solution.plot('B','Time',19);
% solution.plot('Flux Linkage','Time');
% solution.plot('Flux Linkage','Harmonic');
% solution.plot('Torque','Time');
% solution.plot('Torque','Harmonic');
% solution.plot('Voltage','Time');
% solution.plot('Current','Time');
% solution.plot('Current','Harmonic');
