%inlineIf.m Two argument if-then-else function for use with inline expressions
%   valueOut = inlineIf(testBool,trueValue,falseValue) returns trueValue if
%   testBool is true and falseValue if testBool is false.
%
% See also if

function valueOut=inlineIf(testBool,trueValue,falseValue)
    if testBool
        valueOut = trueValue;
    else
        valueOut = falseValue;
    end
end