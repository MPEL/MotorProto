function argNameCell = funargnames(functionIn)
    %% Returns the argument names of the function handle specified in functionIn

    argNameCell = func2str(functionIn);
    argNameCell = regexp(argNameCell,'(','split');
    argNameCell = argNameCell{2};
    argNameCell = regexp(argNameCell,')','split');
    argNameCell = argNameCell{1};
    argNameCell = regexp(argNameCell,',','split');
end