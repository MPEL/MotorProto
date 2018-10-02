classdef IsObsolete
    %IsObsolete.m A mixin for deprecating old classes
    methods
        function obsoleteWarning(oldClass,newClass)
            warning(oldClass,'% is being deprecated. Use %s instead',oldClass,newClass);
        end
    end
end