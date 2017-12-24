%computes the normalized maximum frequency, AC
%maximum_frequency = maxfreq(y)
function fmax = maxfreq(X)
    X = inpaint_nans(X);
    L = length(X);
    M = floor(L/2);
    Y = fft(X - repmat(mean(X), size(X, 1), 1));
    P2 = abs(Y/L);
    P1 = P2(1:M+1, :);
    p = repmat(max(P1), size(P1, 1), 1);
    P1 = P1./p;
    P1(1, :) = [];
    f = (1:M)/L;
    [~, index] = max(P1);
    fmax = f(index);
end