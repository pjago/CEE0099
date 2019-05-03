%substitution of a poly on x through poly a. result is a new poly on y
%y = subsp(x, a)
%Ex: subsp([1 0 1 0], [1 -1]) == [1 -3 4 -2]
function y = subsp(x, a)
    n = length(x);
    m = length(a);
    o = (n-1)*(m-1) + 1;
    y = zeros(n, o);
    for i = 1:(n-1)
        y(i, i:o) = x(i)*convp(a, n-i);
    end
    y = sum(y);
    y(end) = x(end) + y(end);
end