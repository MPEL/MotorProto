function a = signedPolyArea(x, y)
% a = signedArea(x,y) returns the signed area of the polygonal
% contour represented by vectors x and y.  Assumes (x,y) is NOT closed.

% Reference: 
% http://geometryalgorithms.com/Archive/algorithm_0101/algorithm_0101.htm

    x = x - mean(x);
    n = numel(x);
    if n <= 2
        a = 0;
    else
        i = [2:n 1];
        j = [3:n 1 2];
        k = (1:n);
        a = sum(x(i) .* (y(j) - y(k)));
    end
    
    a = a / 2;
end
