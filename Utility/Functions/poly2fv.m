function [f, v] = poly2fv(x, y)
%POLY2FV Convert polygonal region to patch faces and vertices
%
%   [F, V] = POLY2FV(X, Y) converts the polygonal region represented by the
%   contours (X, Y) into a faces matrix, F, and a vertices matrix, V, that
%   can be used with the PATCH function to display the region. If the
%   polygon represented by X and Y has multiple parts, either the
%   NaN-separated vector format or the cell array format may be used. The
%   POLY2FV function creates triangular faces.
%
%   Most Mapping Toolbox functions adhere to the convention that individual
%   contours with clockwise-ordered vertices are external contours and
%   individual contours with counterclockwise-ordered vertices are internal
%   contours. Although the POLY2FV function ignores vertex order, you
%   should follow the convention when creating contours to ensure
%   consistency with other functions.
%
%   Note: This version uses the builtin DelaunayTri class to provide limited
%   functionality for users without the Mapping Toolbox. Only cell array based
%   calls are currently implemented and they must be provided in column
%   vector format.
%
%   Example
%   -------
%   Display a rectangular region with two holes using a single patch
%   object.
%
%       % External contour, rectangle.
%       x1 = [0 0 6 6 0].';
%       y1 = [0 3 3 0 0].';
%      
%       % First hole contour, square.
%       x2 = [1 2 2 1 1].';
%       y2 = [1 1 2 2 1].';
%
%       % Second hole contour, triangle.
%       x3 = [4 5 4 4].';
%       y3 = [1 1 2 1].';
%
%       % Compute face and vertex matrices.
%       [f, v] = poly2fv({x1; x2; x3}, {y1; y2; y3});
%
%       % Display the patch.
%       patch('Faces', f, 'Vertices', v, 'FaceColor', 'r', ...
%             'EdgeColor', 'none');
%       axis off, axis equal
%
%   See also DELAUNAYTRI, INPOLYGON.

    sizes       = cellfun('length',x) - 1;
    nRegions    = length(x);
    nVerts      = sum(sizes);
    constraints = [(1:nVerts).',[nVerts,1:(nVerts-1)].'];
    
    lowerInd  = 1;
    for i = 1:nRegions
        constraints(lowerInd,2)                = lowerInd + sizes(i) - 1;
        constraints(lowerInd + sizes(i) - 1,2) = lowerInd;
        lowerInd                               = lowerInd + sizes(i);
    end
    
    %Turn of DelaunayTri warnings which may occur due to multiple points in the
    %polygon definitions
    s    = cell(3,1);
    s{1} = warning('off','MATLAB:DelaunayTri:ConsConsSplitWarnId');
    s{2} = warning('off','MATLAB:DelaunayTri:DupPtsConsUpdatedWarnId');
    s{3} = warning('off','MATLAB:DelaunayTri:DupPtsWarnId');
    
    D  = DelaunayTri(cell2mat(x),cell2mat(y),constraints);
    
    %Restore the warnings to their original state
    warning(s{1}.state,'MATLAB:DelaunayTri:ConsConsSplitWarnId');
    warning(s{2}.state,'MATLAB:DelaunayTri:DupPtsConsUpdatedWarnId');
    warning(s{3}.state,'MATLAB:DelaunayTri:DupPtsWarnId');
    
    for i = 1:nRegions
        x{i}(end+1) = NaN;
        y{i}(end+1) = NaN;
    end
    x = cell2mat(x);
    y = cell2mat(y);
    
    ic = D.incenters;
    ip = inpolygon(ic(:,1),ic(:,2),x,y);
    f  = D.Triangulation(ip,:);
    v  = D.X;
end