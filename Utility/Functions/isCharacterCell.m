%ISCHARACTERCELL True for cell arrays which contain only character arrays
%   ISNUMERIC(A) returns true if A is a cell array where each cell contains
%   a valid character array.
%
%   Example: Each cell contains a character array
%      myCell = {'a','bc';('def').',['g','h';'i','j']}
%      isCharacterCell(myCell)
%
%   Example: Some cells do not contain a character arrays
%      myCell = {pi,'bc';'def',@(x)(x^2)}
%      isCharacterCell(myCell)
%
%   See also ISCELL, ISCHAR, CELLFUN

function boolOut = isCharacterCell(cellIn)
    if iscell(cellIn)
        n       = ndims(cellIn);
     	boolOut = cellfun(@ischar,cellIn);
        for i = 1:n
            boolOut = all(boolOut);
        end
    else
        boolOut = false;
    end
end