%equivalent to global pwm(k) = u(k)
function writesim(~)
global u pwm k
    pwm(k) = u(k);
end