%TRIANGULARPULSEINV x = triangularPulseInv( a, b, c, y )
function x = triangularPulseInv( a, b, c, y )
    if y > 1 || y < 0
        x = [NaN NaN];
    else
        x = [a + y*(b - a) b + (1 - y)*(c - b)];
    end
end

