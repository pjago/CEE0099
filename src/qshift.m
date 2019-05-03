%shifts and adds signal x, using a poly of backward shifts
%if poly is of order n, and there is no fill, will drop the first n values
%if there is a fill, it could be 0: to fill with zeros OR 1: to circshift
%poly could also be a Lag Operator Polynominal
%y = qshift(x, poly, fill?)
%Ex: diff(x) == qshift(x, [1 -1])
%see also: LagOp
function y = qshift(x, poly, varargin)
    if isa(poly, 'LagOp')
        n = poly.Degree + 1;
        polyarr = zeros([1, n]);
        for i = 1:n
            polyarr(i) = poly.Coefficients{i-1};
        end
        poly = polyarr;
    end
    n = length(poly);
    y = zeros(size(x));
    for i = 1:n
        y = y + poly(i)*wshift('1d', x, -(i-1));
    end
    if nargin == 3
        if varargin{1} == 0
            y(1:n-1) = 0;
        end
    else
       y(1:n-1) = [];
    end
end