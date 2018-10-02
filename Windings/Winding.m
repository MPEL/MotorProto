classdef Winding
    properties
        Phases = 3;
        Slots  = 6;
        Poles  = 2;
    end
    
    properties (Dependent, Abstract)
        WindingDiagram
    end
end