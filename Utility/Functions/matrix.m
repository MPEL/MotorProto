%matrix.m
%   This is a compatability function used in conjunction with the matlab 
%   symbolic toolbox. When converting matrices of symbolic expressions to 
%   character arrays, the resulting strings contain a function matrix() which 
%   simply indicates a matrix is contained in the expression but makes the 
%   resulting expression incompatible with the matlab builtin 'eval'.

function argOut = matrix(argIn)
    argOut = argIn;
end