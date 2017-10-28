%at(x, ...) returns a circular access on x, by ...
%example: x = [1 2; 3 4]; at(x, 3, 4) == 2

function got = at(x, varargin)
    idx = varargin;
    for k = 1:length(idx)
        if idx{k} ~= ':'
            idx{k} = mod(idx{k} - 1, size(x, k)) + 1;
        end
    end
    if ~isempty(x)
        got = x(idx{:});
    else
        got = 0;
    end
end