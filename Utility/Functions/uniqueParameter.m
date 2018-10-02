function s = uniqueParameter(s)
    s        = sort(s);
    ds       = [1;diff(s)];
    ds       = abs(ds);
    isUnique = ds > sqrt(eps);
    s        = s(isUnique);
    error('')
end