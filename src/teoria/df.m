function Gjw = df( d, a, eps )
%RELAYDF Gjw = df( d, a, eps )
%   Result of relay to -1/(it's describing function)
Gjw = -pi*sqrt(a^2 - eps^2)/(4*d) -1j*pi*eps/(4*d);
end
