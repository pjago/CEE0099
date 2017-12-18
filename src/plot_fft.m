%plots the normalized fft removing the DC bias
%[handle, maximum_frequency] = plot_fft(y, sampling_frequency)
function [hf, fmax] = plot_fft(X, Fs, varargin)
    X = inpaint_nans(X);
    L = length(X);
    M = floor(L/2);
    Y = fft(X - repmat(mean(X), size(X, 1), 1));
    P2 = abs(Y/L);
    P1 = P2(1:M+1, :);
    p = repmat(max(P1), size(P1, 1), 1);
    P1 = P1./p;
    P1(1, :) = [];
    f = Fs*(1:M)/L;    
    hf = plot(f,P1,varargin{:}); 
    title('Fourier Normalizado (AC)')
    xlabel('f (Hz)')
    ylabel('| FFT |')
    [~, index] = max(P1);
    fmax = f(index);
end