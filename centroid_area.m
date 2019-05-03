function out = centroid_area( params )
%CENTROID x = centroid( params )
switch params{end}
    case 'triangularPulse'
        [h, a, b, c, x] = params{:};
        if isnan(x(1)) && isnan(x(2))
            area = 0;
            centroid = 0;
        else
            S(1) = (b - x(1))*h;
            C(1) = (b + x(1))/2;
            S(2) = (x(2) - b)*h;
            C(2) = (b + x(2))/2;
            S(3) = (x(1) - a)*h/2;
            C(3) = (a + 2*x(1))/3;
            S(4) = (c - x(2))*h/2;
            C(4) = (c + 2*x(2))/3;
            area = sum(S);
            if area ~= 0
                centroid = sum(C.*S)/area;
            else
                centroid = 0;
            end
        end
end
out = [centroid area];
end