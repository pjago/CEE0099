%computes the maximum frequency using xcorr
%maximum_frequency = maxfreq(y)
function fmax = maxfreq(y)
    x = diff(diff(xcorr(y)));
    l = length(x);
    m = ceil(l/2) + 1;
%     % find max and then min
%     [~, sup] = max(x(m:end));
%     [~, loc] = min(x(m+sup:end));
%     fmax = 2/(loc+sup);
    % find second abs peak
    x = abs(x/max(abs(x(x > 0))).*(x > 0) + x/max(abs(x(x < 0))).*(x < 0));
    [~, pek] = findpeaks(x(m:end), 'NPeaks', 2, 'MinPeakHeight', 0.5);
    fmax = 2/pek(2);
end