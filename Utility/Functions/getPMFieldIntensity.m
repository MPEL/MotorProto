function H = getPMFieldIntensity(solution,mat)
    model       = solution.Model;
    M           = numel(model.Mesh);
    isPMElement = cell(1,M);
    for ID = 1:numel(model.Mesh)
        mesh     = model.Mesh(ID);
        regions  = mesh.Regions;
        nRegions = numel(regions);
        isPM     = false(1,nRegions);

        for i = 1:nRegions
            isPM(i) = isa(regions(i).Material,class(mat));
        end
        isPM = find(isPM);

        elementRegions  = mesh.ElementRegions;
        isPMElement{ID} = false(size(elementRegions));

        for i = 1:numel(isPM)
            isPMElement{ID} = (isPMElement{ID} | (elementRegions == isPM(i)));
        end
    end
    
	N = size(solution.Algorithm.X,2);
	H = solution.getContinuumVariableData('H','Time',1:N);
    for ID = 1:M
        for T = 1:N
            H{ID,T} = H{ID,T}(isPMElement{ID},:);
        end
    end
    H = cell2mat(H);
end