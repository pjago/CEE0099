% applies a low pass filter
% [y, n] = lowpf(x, cutoff_frequency)
function [y, n] = lowpf(x, fc)
    n = floor(sqrt(0.1024 + fc^2)/fc);
    y = filter((1/n)*ones(1,n), 1, x);
end