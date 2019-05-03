%MEMBERSHIP fuzzyfier = membership(cellOfStructs)
function mf = membership(cellOfStructs)
    params = cell(size(cellOfStructs));
    type = cell(size(cellOfStructs));
    fx = cell(size(cellOfStructs));
    fy = cell(size(cellOfStructs));
    for i = 1:length(cellOfStructs)
        s = cellOfStructs{i};
        type{i} = s.type;
        params{i} = s.params;
        aux = num2cell(s.params);
        switch s.type
            case 'triangularPulse'
                [a, b, c] = aux{:};
                fx{i} = @(x) triangularPulse(a, b, c, x);
                fy{i} = @(y) [aux triangularPulseInv(a, b, c, y) s.type];
            %case 'other distributions' @todo
        end
    end
    mf.type = type;
    mf.params = params;
    mf.fx = @(x) cellfun(@(f) f(x), fx, 'UniformOutput', false);
    mf.fy = fy;
end