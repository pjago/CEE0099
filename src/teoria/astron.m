function [ kc, ti, td ] = astron( aGjw, bGjw, w, alpha )
%ASTRON [ kp, ti, td ] = astron( aGjw, bGjw, w, alpha )
%   PID parameters that move Gjw from a to b
ra = abs(aGjw);
rb = abs(bGjw);
oa = angle(aGjw);
ob = angle(bGjw);
kc = rb/ra*cos(ob - oa);
ti = 1/(2*alpha*w)*(tan(ob - oa) + sqrt(4*alpha + tan(ob - oa)^2));
td = alpha*ti;
end

