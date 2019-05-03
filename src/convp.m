%iterates the convolution n times on poly (power of n)
%y = convp(x, n)
%Ex: conv(conv(x, x), x) == convp(x, 3)
function y = convp(x, n)
    assert(n >= 0, 'convp n must be >= 0');
    if n == 0
        y = 1;
    else
        y = x;
    end
    for i = 2:n
       y = conv(y, x);
    end
end