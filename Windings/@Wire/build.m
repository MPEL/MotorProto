%build - Divides an input region into a number of conductors 
%   [C, N, M] = build(W, G, R, T, D, L) uses the properties of the Wire object
%   W to divide the Geometry2D object G into several conducting and 
%   non-conducting Regions. C is an array of Region2D objects representing
%   conducting regions, an array N  of Region2D objects representing 
%   non-conducting regions, and a matrix M, representing the  parallel and 
%   series connections of the conducting regions.
%
%   R is a row vector containing two positions along the x-axis which define
%   a strip in the x-y plane oriented parallel to the y-axis. The intersection
%   of this strip and G define the allowable region for conductors to be placed.
%   T is the number of turns (layers). D sets the conductor dynamics which can
%   be specified using a string or the DynamicsTypes enumeration object. L is a
%   string which is used to name the output regions.
%
%   The columns of M give the parallel connections for each turn, while the rows
%   of M give the series connections between turns. For example, M(:, 1) is the
%   first turn consiting of (possibly) many conductors, while M(1,:) shows the
%   paths that a single conductors takes through the slot.
%
%   See the help for Wire and its subclasses for more information.
%
%   Example: Draw a slot with multiple conductors
%       r  = [0.5 1];
%       dr = 0.05;
%       ns = 72;
%
%       G  = slotTemplate(ns, r(1), r(2), 0.05, 0.01, 0.5, 0.5, 1, 0);
%       R  = [r(1) + dr, inf];
%       T  = 2;
%       D  = DynamicsTypes.Dynamic;
%       L  = 'myWires';
%
%       W                     = CircularWire;
%       W.ConductorMaterial   = CopperExampleMaterial;
%       W.InsulatorMaterial   = Air;
%       W.ConductorDiameter   = 2 * pi * r(1) / ns * 0.5 / 3;
%       W.InsulationThickness = 2 * pi * r(1) / ns * 0.5 / 18;
%
%       [C, N, M] = build(W, G, R, T, D, L);
%
%       figure
%       subplot(2,2,1:2);hold on;
%       title('All Conductors + Slot Outline');
%       plot([0.55, 0.55],[-0.05, 0.05]);
%       legend('Conductor Boundary');
%       plot(C);
%       wireframe(N);
%
%       subplot(2,2,3);
%       title('Turn 1');
%       plot(C(M(:,1)));
%       wireframe(N);
%
%       subplot(2,2,4);
%       title('Conductor 1');
%       plot(C(M(1,:)));
%       wireframe(N);
%
% See also Wire, Slot, slotTemplate