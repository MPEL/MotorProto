%% Self_Excited_Synchronous_Field_Winding_Machine.m
%   Model based on Abdi Zeynu's design
%   Some of the underlying code is experimental, partially implemented

clear all;
close all;

%% Initialize the toolbox
simulation = MotorProto('SESFW');

%% Add components to the model
model  = simulation.Model;
rotor  = model.newAssembly('SESFW Rotor','SelfExcitedSynchronousRotor');
stator = model.newAssembly('SESFW Stator','Stator');

%% Define General Machine Parameters
nPoles               = 4;
nStatorTeethPerPhase = 3;
nStatorTeeth         = 3 * nPoles * nStatorTeethPerPhase;
stackLength          = 0.128;
statorOuterRadius    = 0.0967;
statorInnerRadius    = 0.0562 + 1.4*1e-3; %set airgap length
rotorOuterRadius 	 = 0.0562;
rotorInnerRadius     = 0.0182;
w_e = 150;
% 
% %% Define Stator Geometry and Material Properties
stator.ElectricalFrequency = w_e;
stator.Length              = stackLength;
stator.Poles               = nPoles;
stator.Teeth               = nStatorTeeth;
stator.InnerRadius         = statorInnerRadius;
stator.OuterRadius         = statorOuterRadius;
stator.DefaultMaterial     = ArmcoM15;
stator.SourceType          = 'CurrentSource';
stator.ConnectionType      = 'Wye';
stator.CouplingType        = 'Static';
stator.WindingType         = 'Distributed';
stator.Turns               = 14;
stator.ParallelPaths       = 2;
stator.ConductorMaterial   = Copper;

stator.Slot.ConductorType           = 'Homogenized';
stator.Slot.Conductor.PackingFactor = 0.5;

%% Define stator slot geometry
notchLength = eps;
notchWidth  = 1.00*(1 - 0.0071019 / (2 * pi * statorInnerRadius / nStatorTeeth)); %Tooth Face = 7.1mm

slotLength = (0.07630 - statorInnerRadius)/(statorOuterRadius - statorInnerRadius) - notchLength; %L = 20mm from front of tooth
slotWidth  = 1.00*(1-0.00566 / (2 * pi * statorInnerRadius / nStatorTeeth)); %Tooth Width = 5.8mm

slotAngle = 1;

toothFaceWidth = 0.0013593 / (statorOuterRadius - statorInnerRadius); %1.4mm

[slotOutline, slotNotch] = slotTemplate(nStatorTeeth, statorInnerRadius, statorOuterRadius, notchWidth, notchLength, slotWidth, slotLength, slotAngle, 'auto','InnerSlotShape','rounded','InnerSlotLength',toothFaceWidth,'OuterSlotShape','rounded');

stator.Slot.Shape = slotOutline;

if ~isempty(slotNotch)
    stator.addRegion('slot', slotNotch, Air, DynamicsTypes.Static); 
end

%% Set Rotor Parameters
nRotorFieldSlots = 5 * 4;
nRotorTransformerSlots = 2 * 4;
nRotorTeeth = nRotorFieldSlots + nRotorTransformerSlots;

rotor.Poles               = nPoles;
rotor.Teeth               = nRotorTeeth;
rotor.FieldSlots          = nRotorFieldSlots;
rotor.TransformerSlots    = nRotorTransformerSlots;
rotor.Length              = stackLength;
rotor.ElectricalFrequency = w_e;
rotor.InnerRadius         = rotorInnerRadius;
rotor.OuterRadius         = rotorOuterRadius;
rotor.DefaultMaterial     = ArmcoM15;
rotor.OperatingMode       = OperatingModes.Locked;

rotor.FieldSlot.Turns                   = 45;
rotor.FieldSlot.CouplingType            = 'Static';
rotor.FieldSlot.ConductorType           = 'Homogenized';
rotor.FieldSlot.Conductor.PackingFactor = 0.7; %Based on 18AWG, 45*pi*(5.12e-4)^2

rotor.TransformerSlot.Turns                   = 15;
rotor.TransformerSlot.CouplingType            = 'Static';
rotor.TransformerSlot.ConductorType           = 'Homogenized';
rotor.TransformerSlot.Conductor.PackingFactor = 0.3; %Based on 17AWG, 15*pi*(5.75e-4)^2

R = [0,0];
R(1) = nRotorFieldSlots * rotor.FieldSlot.Turns * stackLength / (rotor.FieldSlot.Conductor.ConductorMaterial.sigma * pi * (5.12e-4)^2);
R(2) = nRotorTransformerSlots * rotor.TransformerSlot.Turns * stackLength / (rotor.TransformerSlot.Conductor.ConductorMaterial.sigma * pi * (5.75e-4)^2);

%% Rotor Circuit Parameters
rotor.Circuits.CouplingType  = 'Static';
rotor.Circuits.ParallelPaths = [1, 1];

rotor.Circuits.Cft = [2.25, 0.65] * 1e-6;
rotor.Circuits.Rft = [4.9, 0.5] - R;%[4.9, 0.5] - R;       %Measured = [4.9,0.5]; rotor.Circuits.Rft = External Resistance
rotor.Circuits.Lft = [2.80, 0.08] * 1e-3;  %Measured = [280, 8]*1e-3]; rotor.Circuits.Lft = External Inductance

v = [0.422;0.4570;0.4730;0.4850;0.505;0.513;0.525;0.534;0.543;0.551;0.591;0.624;0.644;0.664;0.672;0.691;0.703;0.715;0.728;0.802;0.867;0.922;0.972;1.017;1.0640;1.0950;1.1320;1.1600];
i = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,2,3,4,5,6,7,8,9,10,20,30,40,50,60,70,80,90,100].';
figure(1);hold on;
scatter(v,i);


R = polyfit(i(end-8:end), v(end-8:end),1);
R = R(1);

v = v(i < 11);
i = i(i < 11);

vo = 0.422-2*(0.457-0.422);

I = v > vo;
v = v(I);
i = i(I);

N = 6;
A = [];
for j = 2:(N-1)
    A = [v.^j-vo^j-j*vo^(j-1),A];
end

A = diag(i)\A;
x = A\(diag(i)\i);
% x = A\i;

x(N-1) = 0;
n = (N-1):-1:0;
for j = 1:(N-2);
    x(N-1) = x(N-1)-n(j)*x(j)*vo^(n(j)-1);
end

x(N) = 0;
for j = 1:(N-1)
    x(N) = x(N)-x(j)*vo^(n(j));
end

dx      = n.'.*x;
dx(end) = [];
dx(end) = dx(end)-1/R;
vs = roots(dx);
vs = real(vs(1));

v = linspace(vo,vs);
i = x(N);
for j = 1:(N-1)
    i = i + x(j)*v.^n(j);
end
plot(v,i,':k');

didv = x(N-1);
for j = 1:(N-2)
    didv = didv + n(j)*x(j)*v.^(n(j)-1);
end
figure(2);
plot(v,didv);

rotor.Circuits.DiodeParameters = x;
rotor.Circuits.DiodeVoltages   = [vo, vs];

%% Define rotor slot geometry
notchWidth  = 1.6 * 0.002805 / (2 * pi * rotorOuterRadius / nRotorTeeth); %2.805mm
notchLength = eps; %0.563mm

slotLength = (rotorOuterRadius - 0.040186) / (rotorOuterRadius - rotorInnerRadius) - notchLength; %r = 40.186mm
slotWidth  = 1.00 * 0.007478 / (2 * pi * rotorOuterRadius / nRotorTeeth); %7.233mm

toothFaceWidth = 0.06; %r=53.622mm

[slotOutline, slotNotch] = slotTemplate(nRotorTeeth, rotorInnerRadius, rotorOuterRadius, notchWidth, notchLength, slotWidth, slotLength, 1, 0.15, 'InnerSlotShape','straight','InnerSlotLength',toothFaceWidth,'OuterSlotShape','rounded','AirgapLocation','outside');

rotor.SlotShape = slotOutline;
rotor.addRegion('slot', slotNotch, Air, DynamicsTypes.Static); 

%% Set mesh parameters
mesh = simulation.Mesh;
% mesh(1).MaximumElementSize = statorInnerRadius * 2 * pi / nStatorTeeth / 6;
% mesh(2).MaximumElementSize = rotorOuterRadius * 2 * pi / nRotorTeeth / 6;
% mesh(2).UseUniformGrid     = true;

%% Configure algorithm
nTimePoints  = 18;
% simulation.configureAlgorithm('TPFEM', 'TimePoints', nTimePoints, 'RungeKuttaStages', 1, 'StorageLevel', 3, 'Verbose', true, 'NewtonTolerance', 1e-5, 'SymmetricJacobian', true);
simulation.configureAlgorithm('ShootingNewton', 'TimePoints', nTimePoints, 'RungeKuttaStages', 2, 'StoreDecompositions', true, 'Verbose', true, 'MaxGMRESIterations', 10, 'ShootingTolerance', 1e-6, 'NewtonTolerance', 1e-6, 'GMRESTolerance', 1e-6, 'SymmetricJacobian', true,'MaxNewtonIterations',20,'MaxShootingIterations',100);

%% Inductance Calculation
% stator.SourceType            = 'CurrentSource';
% stator.ParallelPaths         = 2;
% stator.Circuits.CouplingType = CouplingTypes.Static;
% 
% rotor.OperatingMode        = OperatingModes.Synchronous;
% rotor.Circuits.OpenCircuit = false;
% 
% model.build;
% mesh.build;
% 
% stator.ElectricalFrequency          = eps;
% rotor.ElectricalFrequency           = eps;
% stator.Circuits.ElectricalFrequency = eps;
% stator.Circuits.HarmonicNumbers     = 1;
% stator.Circuits.HarmonicAmplitudes  = 1e-6;
% stator.Circuits.HarmonicPhases      = 0;
% 
% solution = simulation.run;
    
% mf  = solution.Algorithm.Matrices;
% pp  = mf.PostProcessing;
% exo = mf.Exogenous.Magnetic;
% n1  = size(pp(1).Reduced2Full, 2);
% n2  = size(pp(2).Reduced2Full, 2);
% 
% K  = mf.K(0);
% Ir = mf.Index.Global(1).X(end-3:end-2);
% 
% F = [sparse(n1, 3) -K(1:n1,Ir);exo(2).Ff sparse(n2, 2)];
% T = [sparse(3, n1), pp(2).X2Lambda * pp(2).Reduced2Full;
%       pp(1).X2Lambda * pp(1).Reduced2Full, sparse(2, n2)];
% 
% Ir = mf.Index.Global(1).X(end-3:end);
% 
% F(Ir,:) = [];
% T(:,Ir) = [];
%   
% X = solution.Algorithm.X;
% t = solution.Algorithm.Times;
% N = numel(t) - 1;
% t = t(1:N);
% L = cell(1,1,N);
% for i = 1:N
%     x = [pp(1).Full2Reduced*X{1,i};pp(2).Full2Reduced * X{2,i}];
%     K = mf.K(t(i));
%   	G = mf.G(t(i), x);
%     J = K+G;
%     J(Ir,:) = [];
%     J(:,Ir) = [];
%     
%     L{i} = full(T * (J \ F));
% end
% 
% L = cell2mat(L);
% figure(3);clf;
% hold on;
% title('Three Phase Incremental Inductances');
% plot(t,squeeze(L(1,1,:)));
% plot(t,squeeze(L(2,2,:)),'r-.');
% plot(t,squeeze(L(3,3,:)),'k--');s
% plot(t,squeeze(L(4,4,:)),'b-');
% plot(t,squeeze(L(5,5,:)),'m--');
% xlabel('Time [s]');
% ylabel('Inductance [H]');
% legend('A','B','C','F','T');
% ylim([min(min(min(min(L))),0) max(max(max(L)))]);
% 
% Lm = mean(L,3)
% T = [1 -1 0 0 0;0 1 -1 0 0;-1 0 1 0 0;0 0 0 1 0;0 0 0 0 1];
% T'*Lm*T

%% Open Circuit Voltage Parameter Sweep
stator.SourceType            = 'VoltageSource';
stator.ParallelPaths         = 2;
stator.Circuits.CouplingType = CouplingTypes.Static;

%FFT Based Measurements
vt = [ 7.9617, 11.6052, 14.9654,...
      12.3319, 19.5364, 25.1281];
  
vf = [ 7.5822, 12.0114, 16.1772,...
      12.6439, 20.3789, 27.2112];

I = [3, 3, 3, 5, 5, 5];
       
w_e = [50, 100, 150, 50, 100, 150];

rotor.OperatingMode        = OperatingModes.Locked;
rotor.InitialAngle         = -2*pi/nRotorTeeth*(5+2/2) + 2*pi/(2*nPoles); %Transformer Winding Excitation
rotor.Circuits.OpenCircuit = true;

model.build;
mesh.build;

% V = vt;
% for k = 1:1
%     for j = 1:1
%         %% Adjust frequency
%         stator.ElectricalFrequency = w_e(j);
%         rotor.ElectricalFrequency  = w_e(j);
%         
%         %% Set Excitation
%         stator.Circuits.ElectricalFrequency = w_e(j);
%         stator.Circuits.HarmonicNumbers     = [1;1;1];
%         stator.Circuits.HarmonicAmplitudes  = V(j) * sqrt(2/3) * cos([-1/3;0;1/3]*2*pi+pi);
%         stator.Circuits.HarmonicPhases      = [0;0;0];
%         
%         %% Simulate
%         solution = simulation.run;
%         
%         %% Plot
%         solution.plot('Voltage', 'Time');
%         close(gcf);
%         title(sprintf('Locked Rotor, Open Circuit, Voltage Driven, i = %dA, f = %dHz',I(j), w_e(j)));
%         
%         %% Wait for plotting to finish
%         pause(1);
%     end
%     rotor.InitialAngle = -2*pi/nRotorTeeth*(5/2) + 2*pi/(2*nPoles);%Field Winding Excitation
%     model.build;
%     mesh.build;
%     V = vf;
% end

%% Open Circuit Current Parameter Sweep
% stator.SourceType            = 'CurrentSource';
% stator.ParallelPaths         = 2;
% stator.Circuits.CouplingType = CouplingTypes.Static;
% 
% rotor.OperatingMode          = OperatingModes.Locked;
% rotor.InitialAngle           = -2*pi/nRotorTeeth*(5+2/2) + 2*pi/(2*nPoles); %Transformer Winding Excitation
% rotor.Circuits.OpenCircuit   = true;
% 
% model.build;
% mesh.build;
% 
% V = vt;
% for k = 1:1
%     for j = 1:1
%         %% Adjust frequency
%         stator.ElectricalFrequency = w_e(j);
%         rotor.ElectricalFrequency  = w_e(j);
%         
%         %% Set Excitation
%         stator.Circuits.ElectricalFrequency = w_e(j);
%         stator.Circuits.HarmonicNumbers     = [1;1;1];
%         stator.Circuits.HarmonicAmplitudes  = I(j) * sqrt(2/3) * cos([-1/3;0;1/3]*2*pi+pi);
%         stator.Circuits.HarmonicPhases      = [0;0;0];
%         
%         %% Simulate
%         solution = simulation.run;
%         
%         %% Plot
%         solution.plot('Voltage', 'Time');
%         close(gcf);
%         title(sprintf('Locked Rotor, Open Circuit, Current Driven, i = %dA, f = %dHz',I(j), w_e(j)));
%         
%         %% Wait for plotting to finish
%         pause(1);
%     end
%     rotor.InitialAngle = -2*pi/nRotorTeeth*(5/2) + 2*pi/(2*nPoles);%Field Winding Excitation
%     model.build;
%     mesh.build;
%     V = vf;
% end

%%  Locked Rotor + Rectifier Current Parameter Sweep
stator.SourceType            = 'CurrentSource';
stator.ParallelPaths         = 2;
stator.Circuits.CouplingType = CouplingTypes.Static;
       
I   = [5,10];
w_e = [100,200];

rotor.Circuits.Cft         = [3.7, 0.911]*1e-6; 
rotor.OperatingMode        = OperatingModes.Locked;
rotor.InitialAngle         = -2*pi/nRotorTeeth*(5+2/2) + 2*pi/(2*nPoles); %Transformer Winding Excitation
rotor.Circuits.OpenCircuit = false;

model.build;
mesh.build;

for j = numel(I)
    %% Adjust frequency
    stator.ElectricalFrequency = w_e(j);
    rotor.ElectricalFrequency  = w_e(j);
    
    %% Set Excitation
    stator.Circuits.ElectricalFrequency = w_e(j);
    stator.Circuits.HarmonicNumbers     = [1;1;1];
    stator.Circuits.HarmonicAmplitudes  = I(j) * sqrt(2/3) * cos([-1/3;0;1/3]*2*pi+pi);
    stator.Circuits.HarmonicPhases      = [0;0;0];
    
    %% Simulate
    solution = simulation.run;
    
    %% Plot
    solution.plot('Voltage', 'Time');
    close(gcf);
    title(sprintf('Locked Rotor + Rectifier, i = %dA, f = %dHz', I(j), w_e(j)));
    
    solution.plot('Current', 'Time');
    close(gcf);
    title(sprintf('Locked Rotor + Rectifier, i = %dA, f = %dHz', I(j), w_e(j)));
    
    %% Wait for plotting to finish
%     pause(1);
end

%% Synchronous Operation + Rectifier Current Parameter Sweep
% w_e = 1200 / 60 * nPoles / 2;
% rotor.InitialAngle = -2*pi/nRotorTeeth*(5+2/2) + 2*pi/(2*nPoles); %Transformer Winding Excitation
% 
% stator.SourceType            = 'CurrentSource';
% stator.ParallelPaths         = 2;
% stator.Circuits.CouplingType = CouplingTypes.Static;
% 
% rotor.OperatingMode        = OperatingModes.Synchronous;
% rotor.Circuits.OpenCircuit = false;
% rotor.Circuits.Cft         = [3.7, 0.911] * 1e-6;
% 
% model.build;
% mesh.build;
% 
% % Adjust frequency
% stator.ElectricalFrequency = w_e;
% rotor.ElectricalFrequency  = w_e;
% 
% hns = {[1, 12, 14; 1, 12, 14; 1, 12, 14], [1, 5, 7; 1, 5, 7; 1, 5, 7]};
% hf  = [520, 240];
% for f = 1:1
%     %% Set Excitation
%     Ic  = 8:2:42;
%     tau = zeros(size(Ic));
% 
%     stator.Circuits.ElectricalFrequency = w_e;
%     stator.Circuits.HarmonicNumbers     = hns{f};
%     stator.Circuits.HarmonicPhases      = [-1 -1 -1;0 0 0;1 1 1] * 2 * pi / 3;
% 
%     %% Simulate
%     stator.Circuits.HarmonicAmplitudes = [Ic(1), 7/2, 7/2;Ic(1), 7/2, 7/2;Ic(1), 7/2, 7/2];
%     solution = simulation.run;
%     tau_h = solution.getBulkVariableData('Torque','Harmonic');
%     tau(1) = tau_h{1}(1)
    
%     for i = 2:5:numel(Ic)
%         stator.Circuits.HarmonicAmplitudes  = [Ic(i), 7/2, 7/2;Ic(i), 7/2, 7/2;Ic(i), 7/2, 7/2];
%         solution = simulation.run(solution.Algorithm.Y);
%         tau_h = solution.getBulkVariableData('Torque','Harmonic');
%         tau(i) = tau_h{1}(1)
%     end

%     figure;hold on;
%     plot(Ic,tau);
%     title(sprintf('Torque, Ic = 7, fc = %dHz',hf(f)));
%     xlabel('Io [A]');
%     ylabel('Torque [N-m]');
%     grid on;

    %%
%     Io  = 42;
%     Ic  = [0 1:2:25];
%     tau = zeros(size(Ic));
% 
%     stator.Circuits.ElectricalFrequency = w_e;
%     stator.Circuits.HarmonicNumbers     = hns{f};
%     stator.Circuits.HarmonicPhases      = [-1 -1 -1;0 0 0;1 1 1] * 2 * pi / 3;
% 
%     %% Simulate
%     stator.Circuits.HarmonicAmplitudes = [Io, Ic(1)/2, Ic(1)/2;Io, Ic(1)/2, Ic(1)/2;Io, Ic(1)/2, Ic(1)/2];
%     solution = simulation.run;
%     tau_h = solution.getBulkVariableData('Torque','Harmonic');
%     tau(1) = tau_h{1}(1)
% 
%     for i = 2:6:numel(Ic)
%         stator.Circuits.HarmonicAmplitudes = [Io, Ic(i)/2, Ic(i)/2;Io, Ic(i)/2, Ic(i)/2;Io, Ic(i)/2, Ic(i)/2];
%         solution = simulation.run(solution.Algorithm.Y);
%         tau_h = solution.getBulkVariableData('Torque','Harmonic');
%         tau(i) = tau_h{1}(1)
%     end
% 
%     figure;hold on;
%     plot(Ic,tau);
%     title(sprintf('Torque, Io = 42, fc = %dHz',hf(f)));
%     xlabel('Io [A]');
%     ylabel('Torque [N-m]');
%     grid on;
% end
%% Plotting
% solution.plot('A','Time',1);
% solution.plot('B','Time',1);
% solution.plot('H','Time',1);
% solution.plot('M','Time',1);
% solution.plot('J','Time',1);
% solution.plot('E','Time',1);
% solution.plot('A','Harmonic',1);
% solution.plot('B','Harmonic',[0,1]);
% solution.plot('H','Harmonic',[1]);
% solution.plot('LossDensity', 'UseSinglePlot', true, 'DataFunction', @(x)(log10(x)), 'DataFunctionString', 'log_{10}');
% solution.plot('J','Harmonic',1);
% solution.plot('E','Harmonic',1);
% 
% solution.plot('Flux Linkage','Time');
% solution.plot('Flux Linkage','Harmonic');
%solution.plot('Torque','Time');
% solution.plot('Torque','Harmonic');
% solution.plot('Voltage','Time');
% solution.plot('Voltage','Harmonic');
% solution.plot('Current','Time');
% solution.plot('Current','Harmonic');