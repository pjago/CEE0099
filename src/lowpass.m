% Refência: https://dsp.stackexchange.com/questions/9966/what-is-the-cut-off-frequency-of-a-moving-average-filter

function [y, n] = lowpass(x, fc)
    n = floor(sqrt(0.1024 + fc^2)/fc);
    y = filter((1/n)*ones(1,n), 1, x);
end