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
stator.WindingType         = WindingTypes.Distributed;
stator.ParallelPaths       = nParallelPaths;

%% Slot
stator.Slot.Turns = nTurnsPerSlot;

%% Stranded Conductors
stator.Slot.ConductorType                 = ConductorTypes.Circular;
stator.Slot.Conductor.ConductorDiameter   = 1.0e-3 * 2 * 0.96;
stator.Slot.Conductor.InsulationThickness = 0.10e-3 * 2 * 0.96;

%% Define slot geometry
Bsat = 2.0;
Bmag = 1.23;

slotWidth   = 1-statorInnerRadius/rotorOuterRadius*Bmag/Bsat;
slotLength  = 1-statorInnerRadius*(1-slotWidth)*2*pi/nPoles*(3*nTeethPerPhase-1)/(3*nTeethPerPhase)/(statorOuterRadius-statorInnerRadius)/2;
notchWidth  = 0.1795;
notchLength = 0.01263;
[slotOutline, slotNotch] = slotTemplate(nTeeth, statorInnerRadius,statorOuterRadius, notchWidth, notchLength, slotWidth, slotLength, 1, 'auto','InnerSlotShape','rounded','OuterSlotShape','rounded');

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
rotor.InitialAngle        = 30*pi/180 * (2/nPoles);
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
rotor.addRegion('pm', permanentMagnet,  NdFe35, DynamicsTypes.Static);

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

retainingRing = Geometry2D.draw('Sector', 'Radius', [rotorOuterRadius-pmRing,rotorOuterRadius], 'Angle', 2 * pi / nPoles, 'Rotation', - pi / nPoles,'PlotStyle',{'b'});
rotor.addRegion('ring', retainingRing, Iron, DynamicsTypes.Static);

%% Set mesh parameters
mesh = simulation.Mesh;
mesh(1).MaximumElementSize = pmWidth / 4;
mesh(2).MaximumElementSize = (2*pi*statorInnerRadius)*(0.5/nTeeth)*0.28;
mesh(1).MaximumAirgapEdgeLength = [inf, 2*pi*rotorOuterRadius / nTeeth / 10];
mesh(2).MaximumAirgapEdgeLength = [2*pi*statorInnerRadius / nTeeth / 10, inf];

%% Generate Initial Conditions

%% Open Circuit Static Simulation
stator.CouplingType  = CouplingTypes.Static;
stator.SourceType    = SourceTypes.CurrentSource;
stator.ParallelPaths = nParallelPaths;

stator.Circuits.ElectricalFrequency = f_e;
stator.Circuits.HarmonicNumbers     = 1;
stator.Circuits.HarmonicAmplitudes  = 0;
stator.Circuits.HarmonicPhases      = 0;

model.build;
mesh.build;

simulation.configureAlgorithm('Static', 'TimePoints', 18, 'Verbose', true);
ICSIM = simulation.run;

Lambdah = ICSIM.getBulkVariableData('FluxLinkage', 'Harmonic');

%% Calculate 3-Phase Incremental Inductances
mf  = ICSIM.Algorithm.Matrices;
pp  = mf.PostProcessing;
exo = mf.Exogenous.Magnetic;
n1  = size(pp(1).Reduced2Full, 2);

F = [sparse(n1, 3); exo(2).Ff];
T = [sparse(3, n1), pp(2).X2Lambda * pp(2).Reduced2Full];
X = ICSIM.Algorithm.X;
t = ICSIM.Algorithm.Times;
N = numel(t) - 1;
t = t(1:N);
Labc = cell(1,1,N);

for i = 1:N
    x = [pp(1).Full2Reduced*X{1,i};pp(2).Full2Reduced * X{2,i}];
    K = mf.K(t(i));
    G = mf.G(t(i), x);
    
    Labc{i} = full(T * ((K+G) \ F));
end
L  = cell2mat(Labc);

%% Calculate sinusoidal current magnitude and phase for sine and square wave voltages
L0 = zeros(3,3);
L2 = zeros(3,3);
Lambdah = Lambdah{1};

for i = 1:3
    Lambdah{i} = Lambdah{i}(2);
    for j = 1:3
        Lh = fft(squeeze(L(i,j,:))) / size(L,3);
        L0(i,j) = Lh(1);
        L2(i,j) = Lh(3);
    end
end
Lambdah = cell2mat(Lambdah);
pvec = exp(1i*2*pi*[0;-1/3;1/3]);
L0p  = L0*pvec;
L0p  = L0p(1);
L2p  = L2*conj(pvec);
L2p  = L2p(1);

ph = -Lambdah ./ abs(Lambdah);
ph2 = [ph zeros(3,1);zeros(3,1) conj(ph)];

M = 1i*2*pi*f_e*[-L0 -2*conj(L2);2*L2 L0];
d = (170/sqrt(3)*ph-(1i*2*pi*f_e)*Lambdah);
d = [d;conj(d)];
Isine = (ph2'*M*ph2) \ (ph2'*d);
Isine = ph2*Isine;
Isine = -2*Isine(1);

% DV = (170/sqrt(3)-(1i*2*pi*f_e)*Lambdah(1)) / (1i*2*pi*f_e);
% Ireal = real(DV) / (L0p+L2p+conj(L2p));
% Iimag = imag(DV) / (L0p+conj(L2p)-L2p);
% Isine = 2*(Ireal+1i*Iimag);

O = 2;
for o = 1:O
    %% Sine wave static simulations
    stator.Circuits.ElectricalFrequency = f_e;
    stator.Circuits.HarmonicNumbers     = 1;
    stator.Circuits.HarmonicAmplitudes  = abs(Isine);
    stator.Circuits.HarmonicPhases      = angle(Isine);
    ICSINE = simulation.run;
    
    Lambdah = ICSINE.getBulkVariableData('FluxLinkage', 'Harmonic');
    
    mf  = ICSINE.Algorithm.Matrices;
    pp  = mf.PostProcessing;
    exo = mf.Exogenous.Magnetic;
    n1  = size(pp(1).Reduced2Full, 2);
    
    F = [sparse(n1, 3); exo(2).Ff];
    T = [sparse(3, n1), pp(2).X2Lambda * pp(2).Reduced2Full];
    X = ICSINE.Algorithm.X;
    t = ICSINE.Algorithm.Times;
    N = numel(t) - 1;
    t = t(1:N);
    Labc = cell(1,1,N);
    
    for i = 1:N
        x = [pp(1).Full2Reduced*X{1,i};pp(2).Full2Reduced * X{2,i}];
        K = mf.K(t(i));
        G = mf.G(t(i), x);
        
        Labc{i} = full(T * ((K+G) \ F));
    end
    L  = cell2mat(Labc);
    
    L0 = zeros(3,3);
    L2 = zeros(3,3);
    Lambdah = Lambdah{1};
    
    for i = 1:3
        Lambdah{i} = Lambdah{i}(2);
        for j = 1:3
            Lh = fft(squeeze(L(i,j,:))) / size(L,3);
            L0(i,j) = Lh(1);
            L2(i,j) = Lh(3);
        end
    end
    Lambdah = cell2mat(Lambdah);
    pvec = exp(1i*2*pi*[0;-1/3;1/3]);
    L0p  = L0*pvec;
    L0p  = L0p(1);
    L2p  = L2*conj(pvec);
    L2p  = L2p(1);
    
    M = 1i*2*pi*f_e*[-L0 -2*conj(L2);2*L2 L0];
    d = (170/sqrt(3)*ph-(1i*2*pi*f_e)*Lambdah);
    d = [d;conj(d)];
    d = (ph2'*M*ph2) \ (ph2'*d);
    d = ph2*d;
    
    Isine = Isine - 2*d(1);
    
%     DV = (170/sqrt(3)-(1i*2*pi*f_e)*Lambdah(1)) / (1i*2*pi*f_e);
%     Ireal = real(DV) / (L0p+L2p+conj(L2p));
%     Iimag = imag(DV) / (L0p+conj(L2p)-L2p);
%     Isine = Isine + 2*(Ireal+1i*Iimag);
    
    stator.Circuits.ElectricalFrequency = f_e;
    stator.Circuits.HarmonicNumbers     = 1;
    stator.Circuits.HarmonicAmplitudes  = abs(Isine);
    stator.Circuits.HarmonicPhases      = angle(Isine);
    ICSINE = simulation.run;
    
    if o == O
        Vb = ICSINE.getBulkVariableData('Voltage', 'Time');
        Ib = ICSINE.getBulkVariableData('Current', 'Time');
    end
    
    ICSINE.plot('Current','Time');
    ICSINE.plot('Voltage','Time');
    ICSINE.plot('Voltage','Harmonic');
    ICSINE.plot('Torque','Harmonic');
end

%% Sine and square wave initial conditions
stator.SourceType = 'VoltageSource';
stator.Circuits.ElectricalFrequency = f_e;
stator.CouplingType = CouplingTypes.Dynamic;

rotor.InputRegions(1).Dynamics = DynamicsTypes.Floating;
rotor.BackironType = BackironTypes.Laminated;

model.build;
mesh.build;

sol = ICSINE;
mf_init = ICSINE.Matrices;

x0  = cell(1,2);
X0  = cell(1,2);
for S = 1:1
    solution = sol(S);
    x   = solution.Algorithm.X;
    x_t = solution.Algorithm.X_t;
    
    x0{S}    = x(:,1:18);
    mf_init  = solution.Matrices;
    mf_new   = DynamicMatrixFactory(model);
    assembly = model.Assemblies;
    k        = [0,0];
    for p = 1:18
        for i = 1:numel(assembly)
            regions   = assembly(i).Regions;
            ind       = mf_new.Index.Local(i);
            coupling  = mf_init.Coupling(i);
            
            circuit = assembly(i).Circuits;
            if numel(circuit) > 0
                Rk = circuit.RegionSets;
                Rs = circuit.RegionPolarity;
                Ps = circuit.PathSets;
                
                I = circuit.ScalarPotentialIndex;
                J = circuit.StrandCurrentIndex;
                K = circuit.BundleVoltageIndex;
                L = circuit.BundleCurrentIndex;
                
                Nb = numel(Rk);
                for j = 1:Nb
                    Nsb = numel(Rk{j});
                    for k = 1:Nsb
                        for l = 1:numel(Rk{j}{k})
                            m = Rk{j}{k}(l);
                            s = Rs{j}{k}(l);
                            n = I{j}{k}(l);
                            
                            v = - coupling.Integral{m} * x_t{i,p} / coupling.Area(m);
                            v = v - s * Ib{1}{j}(p) / (coupling.Conductivity(m) * coupling.Area(m)) / numel(Ps{j});
                            
                            x0{S}{i,p}(n) = v;
                        end
                        
                        n = J{j}(k);
                        v = Ib{1}{j}(p) / numel(Ps{j});
                        
                        x0{S}{i,p}(n) = v;
                    end
                    
                    x0{S}{i,p}(K(j)) = Vb{1}{j}(p);
                    x0{S}{i,p}(L(j)) = Ib{1}{j}(p);
                end
                Vabc = cell2mat(Vb{1});
                Vabc = Vabc(:,p);
                
                x0{S}{i,p}(end+1) = -sum(Vabc)/3;
                x0{S}{i,p}        = mf_new.PostProcessing(i).Full2Reduced * x0{S}{i,p};
            else
                x0{S}{i,p} = mf_init.PostProcessing(i).Full2Reduced * x0{S}{i,p};
            end
            
            for j = 1:numel(regions)
                if regions(j).Dynamics == DynamicsTypes.Floating
                    l = ind.Regions(j);
                    v = coupling.Integral{j} * x_t{i,p} / coupling.Area(j);
                    x0{S}{i,p}(l) = -v;
                end
            end
        end
    end
    x0{S} = cell2mat(x0{S});
    X0{S} = fft(x0{S}, [], 2) / 18;
    X0{S}(:,9) = [];
end
M = [0:8, -8:-1]';

%% Test Simulations, Sine
% stator.SourceType = SourceTypes.VoltageSource;
% stator.ParallelPaths = nParallelPaths;
% stator.Circuits.ElectricalFrequency = f_e;
% stator.Circuits.HarmonicNumbers     = 1;
% stator.Circuits.HarmonicAmplitudes  = 340 / sqrt(3);
% stator.Circuits.HarmonicPhases      = 0;
%
% simulation.configureAlgorithm('ShootingNewton',  'TimePoints', 42,'RungeKuttaStages', 2, 'StoreDecompositions', true, 'Verbose', true,'MaxGMRESIterations', 50, 'ShootingTolerance', 1e-6, 'NewtonTolerance', 1e-6,'GMRESTolerance', 1e-3, 'SymmetricJacobian',true,'MaxNewtonIterations',20,'MaxShootingIterations',10,'Adaptive',false,'AdaptiveTolerance',1e-5);
% solution = simulation.run(x0{1}(:,1));
%
% solution.plot('Voltage','Harmonic');
% solution.plot('Torque','Harmonic');
% solution.plot('Current','Harmonic');

%% Test Simulations, Square
% h = 1:2:1001;
% V = 340 / 2 * 4/pi./h .* abs(1./(1+(1i*h*f_e/12000)));
% stator.SourceType = SourceTypes.VoltageSource;
% stator.ParallelPaths = nParallelPaths;
% stator.Circuits.ElectricalFrequency = f_e;
% stator.Circuits.HarmonicNumbers     = h;
% stator.Circuits.HarmonicAmplitudes  = abs(V);
% stator.Circuits.HarmonicPhases      = angle(V);
%
% simulation.configureAlgorithm('ShootingNewton',  'TimePoints', 42,'RungeKuttaStages', 2, 'StoreDecompositions', true, 'Verbose', true,'MaxGMRESIterations', 50, 'ShootingTolerance', 1e-6, 'NewtonTolerance', 1e-6,'GMRESTolerance', 1e-3, 'SymmetricJacobian',true,'MaxNewtonIterations',20,'MaxShootingIterations',10,'Adaptive',false,'AdaptiveTolerance',1e-5);
% solution = simulation.run(x0{2}(:,1));
%
% solution.plot('Voltage','Harmonic');
% solution.plot('Torque','Harmonic');
% solution.plot('Current','Harmonic');

%% Parameter Sweep Simulation
% for w = 1:1
%     switch w
%         case 1
%             atol = {10.^(-(2:6))};
%             atol = repmat(atol,14,1);
%             atol{3} = 10.^(-(2:5));
%             for i = 1:3
%                 atol{3+i} = atol{3};
%             end
% 
%             filename = sprintf('C:\\Users\\Jason\\Dropbox\\SteadyStateShootoutSine2.mat');
%             NSteps    = cell(14,length(atol{1}));
%             SimTime   = cell(14,length(atol{1}));
%             CondLoss  = cell(14,length(atol{1}));
%             CoreLoss  = cell(14,length(atol{1}));
%             Aharmonic = cell(14,length(atol{1}));
%             DiscErr   = cell(14,length(atol{1}));
%             
%             stator.SourceType = SourceTypes.VoltageSource;
%             stator.ParallelPaths = nParallelPaths;
%             stator.Circuits.ElectricalFrequency = f_e;
%             stator.Circuits.HarmonicNumbers     = 1;
%             stator.Circuits.HarmonicAmplitudes  = 340 / sqrt(3);
%             stator.Circuits.HarmonicPhases      = 0;
%         case 2
%             atol = {10.^(-(1:5))};
%             atol = repmat(atol,14,1);
%             atol{3} = 10.^(-(1:4));
%             for i = 1:3
%                 atol{3+i} = atol{3};
%             end
%             
%             filename = sprintf('C:\\Users\\Jason\\Dropbox\\SteadyStateShootoutSquare2.mat');
%             NSteps    = cell(14,length(atol{1}));
%             SimTime   = cell(14,length(atol{1}));
%             CondLoss  = cell(14,length(atol{1}));
%             CoreLoss  = cell(14,length(atol{1}));
%             Aharmonic = cell(14,length(atol{1}));
%             DiscErr   = cell(14,length(atol{1}));
%             
%             h = 1:2:2001;
%             V = 1i*340 / 2 * 4/pi./h .* abs(1./(1+(1i*h*f_e/12000))).*exp(-1i*pi*h/2);
%             stator.SourceType = SourceTypes.VoltageSource;
%             stator.ParallelPaths = nParallelPaths;
%             stator.Circuits.ElectricalFrequency = f_e;
%             stator.Circuits.HarmonicNumbers     = h;
%             stator.Circuits.HarmonicAmplitudes  = abs(V);
%             stator.Circuits.HarmonicPhases      = angle(V);
%     end
%     pause(10);
%     
%     j = 1;
%     for i = 1:numel(atol{j})
%         simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', 6,'StoreDecompositions', true, 'Adaptive', true, 'AdaptiveTolerance', atol{j}(i));
%         solution = simulation.run;
%         SimTime{j,i} = solution.Algorithm.SimulationTime;
%         DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%         CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%         CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%         Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%         NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%         clear solution;
%         pause(10);
%     end
%     save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%     pause(10);
%     
%     j = j+1;
%     for i = 1:numel(atol{j})
%         t = model.getTimePoints(NSteps{j-1,i});
%         simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', length(t)-1, 'StoreDecompositions', true, 'Adaptive', false);
%         solution = simulation.run;
%         SimTime{j,i} = solution.Algorithm.SimulationTime;
%         DiscErr{j,i} = DiscErr{j-1,i};
%         CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%         CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%         Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%         NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%         clear solution;
%         pause(10);
%     end
%     save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%     pause(10);
%     
%     for stages = 1:3
%         j = j + 1;
%         for i = 1:numel(atol{j})
%             simulation.configureAlgorithm('ShootingNewton',  'TimePoints', 18,'RungeKuttaStages', stages, 'StoreDecompositions', true, 'SymmetricJacobian', true, 'Adaptive', true, 'AdaptiveTolerance', atol{j}(i));
%             solution = simulation.run;
%             SimTime{j,i} = solution.Algorithm.SimulationTime;
%             DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%             CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%             CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%             Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%             NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%             clear solution;
%             pause(10);
%         end
%         save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%         pause(10);
%         
%         j = j + 1;
%         for i = 1:numel(atol{j})
%             simulation.configureAlgorithm('ShootingNewton',  'TimePoints', NSteps{j-1,i}, 'RungeKuttaStages', stages, 'StoreDecompositions', true,'SymmetricJacobian', true, 'Adaptive', false, 'AdaptiveTolerance', atol{j}(i));
%             solution = simulation.run;
%             SimTime{j,i} = solution.Algorithm.SimulationTime;
%             DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%             CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%             CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%             Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%             NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%             clear solution;
%             pause(10);
%         end
%         save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%         pause(10);
%         
%         j = j + 1;
%         for i = 1:numel(atol{j})
%             simulation.configureAlgorithm('TPFEM', 'TimePoints', 18,'RungeKuttaStages', stages, 'StoreDecompositions', true, 'SymmetricJacobian', true,'Adaptive', true, 'AdaptiveTolerance', atol{j}(i));
%             solution = simulation.run;
%             SimTime{j,i} = solution.Algorithm.SimulationTime;
%             DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%             CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%             CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%             Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%             NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%             clear solution;
%             pause(10);
%         end
%         save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%         pause(10);
%         
%         j = j + 1;
%         for i = 1:numel(atol{j})
%             simulation.configureAlgorithm('TPFEM', 'TimePoints', NSteps{j-1,i},'RungeKuttaStages', stages, 'StoreDecompositions', true, 'SymmetricJacobian', true,'Adaptive', false, 'AdaptiveTolerance', atol{j}(i));
%             solution = simulation.run;
%             SimTime{j,i} = solution.Algorithm.SimulationTime;
%             DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%             CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%             CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%             Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%             NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%             clear solution;
%             pause(10);
%         end
%         
%         save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%         pause(10);
%     end
% end
% 
% %% Exact Solution
% for w = 1
%     switch w
%         case 1
%             atol = 10^-9;
% 
%             filename = sprintf('C:\\Users\\Jason\\Dropbox\\SineExact2.mat');
%             
%             stator.SourceType = SourceTypes.VoltageSource;
%             stator.ParallelPaths = nParallelPaths;
%             stator.Circuits.ElectricalFrequency = f_e;
%             stator.Circuits.HarmonicNumbers     = 1;
%             stator.Circuits.HarmonicAmplitudes  = 340 / sqrt(3);
%             stator.Circuits.HarmonicPhases      = 0;
%         case 2
%             atol = 10^-9;
%             
%             filename = sprintf('C:\\Users\\Jason\\Dropbox\\SquareExact2.mat');
%             
%             h = 1:2:2001;
%             V = 1i*340 / 2 * 4/pi./h .* abs(1./(1+(1i*h*f_e/12000))).*exp(-1i*pi*h/2);
%             stator.SourceType = SourceTypes.VoltageSource;
%             stator.ParallelPaths = nParallelPaths;
%             stator.Circuits.ElectricalFrequency = f_e;
%             stator.Circuits.HarmonicNumbers     = h;
%             stator.Circuits.HarmonicAmplitudes  = abs(V);
%             stator.Circuits.HarmonicPhases      = angle(V);
%     end
%     pause(10);
%     
%     simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', 6,'StoreDecompositions', false, 'Adaptive', true, 'AdaptiveTolerance', atol);
%     solution = simulation.run;
%     SimTime = solution.Algorithm.SimulationTime;
%     DiscErr = solution.Algorithm.DiscretizationError;
%     CondLoss = solution.getBulkVariableData('AverageConductionLosses');
%     CoreLoss = solution.getBulkVariableData('AverageCoreLosses');
%     Aharmonic = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%     NSteps = numel(solution.Algorithm.Times)-1;
%     clear solution;
% 
%     save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%     pause(10);
% end
% 
% %% Sim Time Factors
% stages = 3;
% for w = 1
%     i = 1;
%     NSteps    = cell(8,8);
%     SimTime   = cell(8,8);
%     CondLoss  = cell(8,8);
%     CoreLoss  = cell(8,8);
%     Aharmonic = cell(8,8);
%     DiscErr   = cell(8,8);
%     for bitype = [BackironTypes.Laminated, BackironTypes.Solid]
%         rotor.BackironType = bitype;
%         model.build;
%         mesh.build;
%         switch w
%             case 1
%                 atol = 1e-3;
% 
%                 filename = sprintf('C:\\Users\\Jason\\Dropbox\\SimTimeFactorsSine2.mat');
% 
%                 stator.SourceType = SourceTypes.VoltageSource;
%                 stator.ParallelPaths = nParallelPaths;
%                 stator.Circuits.ElectricalFrequency = f_e;
%                 stator.Circuits.HarmonicNumbers     = 1;
%                 stator.Circuits.HarmonicAmplitudes  = 340 / sqrt(3);
%                 stator.Circuits.HarmonicPhases      = 0;
%             case 2
%                 atol = 1e-3;
% 
%                 filename = sprintf('C:\\Users\\Jason\\Dropbox\\SimTimeFactorsSquare2.mat');
% 
%                 h = 1:2:2001;
%                 V = 1i*340 / 2 * 4/pi./h .* abs(1./(1+(1i*h*f_e/12000))).*exp(-1i*pi*h/2);
%                 stator.SourceType = SourceTypes.VoltageSource;
%                 stator.ParallelPaths = nParallelPaths;
%                 stator.Circuits.ElectricalFrequency = f_e;
%                 stator.Circuits.HarmonicNumbers     = h;
%                 stator.Circuits.HarmonicAmplitudes  = abs(V);
%                 stator.Circuits.HarmonicPhases      = angle(V);
%         end
%         pause(10);
%         
%         for init = [false,true]
%             for store = [true,false]
%                 j = 1;
%                 if init
%                     t = model.getTimePoints(6);
%                     t(end) = [];
%                     D = exp(1i*2*pi*f_e*M*t);
%                     x_init = real(X0{w}*D);
%                     simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', 6, 'StoreDecompositions', store, 'Adaptive', true, 'AdaptiveTolerance', atol);
%                     solution = simulation.run(x_init);
%                 else
%                     simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', 6, 'StoreDecompositions', store, 'Adaptive', true, 'AdaptiveTolerance', atol);
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%                 
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
%                 
%                 t = model.getTimePoints(NSteps{j-1,i});
%                 if init                          
%                     t(end) = [];
%                     D = exp(1i*2*pi*f_e*M*t);
%                     x_init = real(X0{w}*D);
%                     simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', length(t),'StoreDecompositions', store, 'Adaptive', false);
%                     solution = simulation.run(x_init);
%                 else
%                     simulation.configureAlgorithm('HarmonicBalance', 'TimePoints', length(t)-1,'StoreDecompositions', store, 'Adaptive', false);
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%     
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
%                 
%                 simulation.configureAlgorithm('ShootingNewton',  'TimePoints', 18,'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true, 'Adaptive', true, 'AdaptiveTolerance', atol);
%                 if init
%                     solution = simulation.run(x0{w}(:,1));
%                 else
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%     
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
%                 
%                 simulation.configureAlgorithm('ShootingNewton',  'TimePoints', NSteps{j-1,i}, 'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true, 'Adaptive', false, 'AdaptiveTolerance', atol);
%                 if init
%                     solution = simulation.run(x0{w}(:,1));
%                 else
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%     
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
%                 
%                 simulation.configureAlgorithm('TPFEM', 'TimePoints', 18, 'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true,'Adaptive', true, 'AdaptiveTolerance', atol);
%                 if init                
%                     t = linspace(0,1/f_e,18+1);
%                     h = t(2)-t(1);
%                     [~,~,c] = TPFEM.getButcherTable(stages);
%                     tt = t + h * c(1);
%                     for l = 2:numel(c)
%                         tt = [tt;t + h*c(l)];
%                     end
%                     tt = reshape(tt, 1, []);
%                     D = exp(1i*2*pi*f_e*M*tt);
%                     x_init = real(X0{w}*D);
%                     x_init = mat2cell(x_init, size(x_init,1), ones(1,size(x_init,2)));
%                     x_init = reshape(x_init,numel(c),[]);
%                     solution = simulation.run(x_init);
%                 else
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%     
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
% 
%                 simulation.configureAlgorithm('TPFEM', 'TimePoints', NSteps{j-1,i},'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true,'Adaptive', false, 'AdaptiveTolerance', atol);
%                 if init
%                     t = model.getTimePoints(NSteps{j-1,i});
%                     h = t(2)-t(1);
%                     [~,~,c] = TPFEM.getButcherTable(stages);
%                     tt = t + h * c(1);
%                     for l = 2:numel(c)
%                         tt = [tt;t + h*c(l)];
%                     end
%                     tt = reshape(tt, 1, []);
%                     D = exp(1i*2*pi*f_e*M*tt);
%                     x_init = real(X0{w}*D);
%                     x_init = mat2cell(x_init, size(x_init,1), ones(1,size(x_init,2)));
%                     x_init = reshape(x_init,numel(c),[]);
%                     solution = simulation.run(x_init);
%                 else
%                     solution = simulation.run;
%                 end
%                 SimTime{j,i} = solution.Algorithm.SimulationTime;
%                 DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                 CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                 CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                 Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                 NSteps{j,i} = numel(solution.Algorithm.Times)-1;
%     
%                 clear solution;
%                 pause(10);
%                 j = j + 1;
%                 
%                 %% Transient Analysis
%                 if ~store
%                    	simulation.configureAlgorithm('ShootingNewton',  'TimePoints', 18,'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true, 'Adaptive', true, 'AdaptiveTolerance', atol,'MaxGMRESIterations',0);
%                     if init
%                         solution = simulation.run(x0{w}(:,1));
%                     else
%                         solution = simulation.run;
%                     end
%                     SimTime{j,i} = solution.Algorithm.SimulationTime;
%                     DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                     CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                     CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                     Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                     NSteps{j,i} = numel(solution.Algorithm.Times)-1;
% 
%                     clear solution;
%                     pause(10);
%                     j = j + 1;
% 
%                     simulation.configureAlgorithm('ShootingNewton',  'TimePoints', NSteps{j-1,i}, 'RungeKuttaStages', stages, 'StoreDecompositions', store, 'SymmetricJacobian', true, 'Adaptive', false, 'AdaptiveTolerance', atol,'MaxGMRESIterations',0);
%                     if init
%                         solution = simulation.run(x0{w}(:,1));
%                     else
%                         solution = simulation.run;
%                     end
%                     SimTime{j,i} = solution.Algorithm.SimulationTime;
%                     DiscErr{j,i} = solution.Algorithm.DiscretizationError;
%                     CondLoss{j,i} = solution.getBulkVariableData('AverageConductionLosses');
%                     CoreLoss{j,i} = solution.getBulkVariableData('AverageCoreLosses');
%                     Aharmonic{j,i} = solution.getContinuumVariableData('A','Harmonic',[0,1]);
%                     NSteps{j,i} = numel(solution.Algorithm.Times)-1;
% 
%                     clear solution;
%                     pause(10);
%                     j = j + 1;
%                 end
%                 i = i + 1;
%                 
%                 save(filename, 'SimTime', 'DiscErr', 'CondLoss', 'CoreLoss', 'Aharmonic','NSteps');
%                 pause(10);
%             end
%         end
%     end
% end